#!/usr/bin/env python3
"""
black_line_follow.py — Seguimiento de línea negra sobre fondo claro.

Programa independiente. Reutiliza: rover.py (PWM), encoder_client.py +
encodered (encoders), speed_control.py (PID), imu.py, color_sensor.py,
drive_loop.py (bucle único) y line_control.py (borde, mezcla, estados).

Lo específico de este archivo:
  * Modelo de superficie negra/clara: cobertura = (fondo - C) / (fondo - negro)
    usando el canal "clear" (intensidad total) del TCS34725, calibrado.
  * Calibración manual (sensor sobre negro y sobre fondo) o automática
    (el rover gira sobre sí mismo encima de la línea y registra mínimo/máximo).
  * Orquestación: calibración IMU -> calibración superficie -> seguimiento.

Uso típico:
  python3 lab/black_line_follow.py --calibrate          # una vez por pista/iluminación
  python3 lab/black_line_follow.py --no-motors          # empujar a mano y ver estados
  python3 lab/black_line_follow.py                      # seguir la línea
  python3 lab/black_line_follow.py --auto-calibrate     # calibrar girando y seguir

Colocación inicial: sensor sobre el BORDE de la línea que indica EDGE_SIDE
(por defecto el izquierdo en el sentido de avance), rover alineado con la línea.
"""

import argparse
import sys
import time

import config as _config
from drive_loop import Command, SafetyStop, build_runtime
from line_control import (CalibrationError, ConvoyController, LineFollowStateMachine, ObstacleGuard, States,
                          SurfaceCalibration)


class BlackLineFollower:
    """Paso de control para drive_loop.RoverRuntime.run()."""

    def __init__(self, calibration, cfg=None, imu_feedback=True, events=None):
        self.cfg = cfg or _config
        self.calibration = calibration
        self.machine = LineFollowStateMachine(self.cfg, imu_feedback=imu_feedback, events=events)

    def __call__(self, ctx):
        r = ctx.reading
        if r is None:
            return Command(0, 0, States.STOPPED, stop_reason="sin lectura del sensor de color")
        cov = self.calibration.coverage(r)
        cmd = self.machine.step(cov, ctx, fresh=getattr(r, "fresh", True))
        cmd.log.setdefault("raw", r.value(self.calibration.channel))
        return cmd


class AutoCalibration:
    """Gira sobre sí mismo +S, -S y vuelve a 0 registrando lecturas (estado CALIBRATING)."""

    def __init__(self, mixer, cfg=None, timeout_s=15.0):
        self.cfg = cfg or _config
        self.mixer = mixer
        self.values = []
        self.phase = 0
        self.start_yaw = None
        self.timeout_s = timeout_s

    def __call__(self, ctx):
        c = self.cfg
        if self.start_yaw is None:
            self.start_yaw = ctx.yaw_deg
        if ctx.reading is not None and ctx.reading.fresh:
            self.values.append(ctx.reading.value(c.SURFACE_CHANNEL))
        rel = ctx.yaw_deg - self.start_yaw
        s = c.AUTO_CALIBRATION_SWEEP_DEG
        if self.phase == 0 and rel >= s:
            self.phase = 1
        elif self.phase == 1 and rel <= -s:
            self.phase = 2
        elif self.phase == 2 and rel >= 0:
            return Command(0, 0, States.CALIBRATING, stop_reason="calibración completada")
        if ctx.t > self.timeout_s:
            return Command(0, 0, States.CALIBRATING, stop_reason="calibración: tiempo agotado")
        w = c.AUTO_CALIBRATION_OMEGA_DPS * (1 if self.phase in (0, 2) else -1)
        left, right = self.mixer.mix(0.0, w, ctx.omega_meas, allow_reverse=True)
        return Command(left, right, States.CALIBRATING,
                       {"omega_ref": w, "note": f"calib phase={self.phase} rel={rel:+.0f}"})


def manual_calibration(sensor, cfg, samples=60):
    def collect(label):
        input(f"Colocad el sensor sobre {label} y pulsad Enter...")
        vals = []
        while len(vals) < samples:
            r = sensor.read()
            if r.fresh:
                if r.saturated:
                    print("  AVISO: lectura saturada; reducid COLOR_GAIN o COLOR_INTEGRATION_MS")
                vals.append(r.value(cfg.SURFACE_CHANNEL))
            time.sleep(0.005)
        lo, hi = min(vals), max(vals)
        print(f"  {label}: mediana={sorted(vals)[len(vals)//2]:.0f} (min {lo:.0f}, max {hi:.0f})")
        return vals

    line = collect("la LÍNEA NEGRA (centrado)")
    bg = collect("el FONDO CLARO")
    return SurfaceCalibration.from_samples(line, bg, cfg.SURFACE_CHANNEL, "manual").validate(cfg)


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--calibrate", action="store_true", help="calibración manual y salir")
    ap.add_argument("--auto-calibrate", action="store_true", help="calibrar girando sobre la línea y luego seguir")
    ap.add_argument("--no-motors", action="store_true", help="todo el pipeline sin mover motores (diagnóstico)")
    ap.add_argument("--no-imu", action="store_true", help="no usar IMU (odometría de encoders)")
    ap.add_argument("--perfil", choices=["suave", "rapido"], default="suave",
                    help="suave: menos zigzag; rapido: ~30%% más velocidad con más oscilación")
    ap.add_argument("--no-distance", action="store_true", help="sin parada por obstáculo (sensor de distancia)")
    ap.add_argument("--convoy", action="store_true",
                    help="seguir al robot de delante a CONVOY_TARGET_CM (requiere calibrar el sensor de distancia)")
    ap.add_argument("--edge", choices=["left", "right"], help="borde a seguir (sobrescribe EDGE_SIDE)")
    ap.add_argument("--base-rpm", type=float, help="velocidad de avance (sobrescribe BASE_RPM)")
    ap.add_argument("--max-time", type=float, default=None, help="duración máxima en segundos")
    ap.add_argument("--quiet", action="store_true", help="sin línea de estado periódica")
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    overrides = dict(_config.PROFILES.get(args.perfil, {}))
    if args.edge:
        overrides["EDGE_SIDE"] = args.edge
    if args.base_rpm is not None:
        overrides["BASE_RPM"] = args.base_rpm
    if args.no_imu:
        overrides["IMU_MODEL"] = "none"
    if args.no_motors:
        overrides["STALL_DETECTION"] = False
    if args.no_distance:
        overrides["DISTANCE_ENABLED"] = False
    if args.convoy:
        overrides.update(SEARCH_FIRST_SWEEP_DEG=_config.CONVOY_SEARCH_FIRST_SWEEP_DEG,
                         SEARCH_SECOND_SWEEP_DEG=_config.CONVOY_SEARCH_SECOND_SWEEP_DEG,
                         SEARCH_TIMEOUT_S=_config.CONVOY_SEARCH_TIMEOUT_S)
    cfg = _config.as_namespace(**overrides)

    try:
        rt = build_runtime("black_line", cfg, need_color=True, console=not args.quiet, motors=not args.no_motors,
                           need_distance=not args.calibrate)
    except Exception as e:
        print(f"No se pudo iniciar: {e}")
        return 1

    try:
        if args.calibrate:
            cal = manual_calibration(rt.color, cfg)
            cal.save(cfg.SURFACE_CALIBRATION_FILE)
            print(f"Guardada en {cfg.SURFACE_CALIBRATION_FILE}: {cal}")
            return 0

        print("Calibrando IMU: no mováis el rover...")
        rt.calibrate_imu()
        imu_fb = rt.imu is not None

        if args.auto_calibrate:
            from line_control import DriveMixer
            print("Calibración automática: el rover girará sobre sí mismo encima de la línea.")
            ac = AutoCalibration(DriveMixer(cfg, imu_fb), cfg)
            reason = rt.run(ac, drive_enabled=not args.no_motors)
            if reason != "calibración completada":
                print(f"Calibración automática fallida: {reason}")
                return 1
            cal = SurfaceCalibration.from_sweep(ac.values, cfg.SURFACE_CHANNEL).validate(cfg)
            cal.save(cfg.SURFACE_CALIBRATION_FILE)
            print(f"Calibración guardada: {cal}")
            time.sleep(0.5)
        else:
            cal = SurfaceCalibration.load(cfg.SURFACE_CALIBRATION_FILE).validate(cfg)
        print(f"Usando {cal}. Borde seguido: {cfg.EDGE_SIDE}. IMU: {rt.imu.name if rt.imu else 'no (odometría)'}")

        rt.reset_yaw()
        follower = BlackLineFollower(cal, cfg, imu_feedback=imu_fb, events=rt.logger.event)
        rt.on_imu_disabled = lambda: setattr(follower.machine.mixer, "imu_feedback", False)
        step = follower
        if args.convoy:
            if rt.distance is None:
                print("ERROR: --convoy necesita el sensor de distancia (no uséis --no-distance)")
                return 1
            from distance_sensor import DistanceCalibration
            try:
                dcal = DistanceCalibration.load(cfg.DISTANCE_CALIBRATION_FILE)
            except (FileNotFoundError, ValueError) as e:
                print(f"ERROR: {e}")
                return 1
            step = ConvoyController(follower, follower.machine, dcal, cfg, events=rt.logger.event)
            print(f"CONVOY: separación objetivo {cfg.CONVOY_TARGET_CM:.0f} cm, parada si < {cfg.CONVOY_STOP_CM:.0f} cm, "
                  f"máximo {cfg.CONVOY_MAX_RPM:.0f} RPM")
        elif rt.distance is not None:
            step = ObstacleGuard(follower, cfg, events=rt.logger.event, on_resume=follower.machine.resume)
            print(f"Parada por obstáculo activa (>= {cfg.OBSTACLE_VOLTAGE:.2f} V ≈ 20 cm)")
        else:
            print("Parada por obstáculo DESACTIVADA")
        reason = rt.run(step, max_time=args.max_time, drive_enabled=not args.no_motors)
        print(f"Fin: {reason}")
        if rt.logger.path:
            print(f"Log: {rt.logger.path}")
        return 0
    except (CalibrationError, SafetyStop) as e:
        print(f"ERROR: {e}")
        return 1
    finally:
        rt.close()


if __name__ == "__main__":
    sys.exit(main())
