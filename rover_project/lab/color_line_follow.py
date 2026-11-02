#!/usr/bin/env python3
"""
color_line_follow.py — Seguimiento de una línea de tres colores.

IMPORTANTE (discrepancia documentada en el README): el código original que
funcionaba con los tres colores NO se ha proporcionado. Este programa NO es
una copia de aquel algoritmo; es una reconstrucción sobre la misma
arquitectura (encodered + PID + IMU + bucle único) que:

  * calibra el fondo y cada color (COLOR_NAMES) con el propio sensor;
  * calcula, para cada color, qué fracción de la zona de medida lo cubre
    (proyección del vector RGBC entre "fondo" y "color", normalizado por el fondo);
  * usa esa cobertura con el mismo seguidor de borde y la misma máquina de
    estados que black_line_follow.py (curvas en ambos sentidos, búsqueda acotada);
  * aplica una acción configurable por color (COLOR_ACTIONS: escala de
    velocidad o parada).

Si recuperáis el script original, colocadlo en lab/legacy/ y comparad sus
umbrales y acciones con COLOR_ACTIONS antes de sustituir este archivo.

Uso:
  python3 lab/color_line_follow.py --calibrate
  python3 lab/color_line_follow.py [--no-motors] [--max-time 60]
"""

import argparse
import json
import os
import sys
import time

import config as _config
from drive_loop import Command, SafetyStop, build_runtime
from line_control import CalibrationError, LineFollowStateMachine, ObstacleGuard, States


def _vec(r):
    return [float(r.c), float(r.r), float(r.g), float(r.b)]


class ColorModel:
    def __init__(self, background, colors):
        """background: [c,r,g,b]; colors: {nombre: [c,r,g,b]}"""
        self.bg = [max(1.0, v) for v in background]
        self.colors = {k: v for k, v in colors.items()}
        self._dirs = {}
        for name, v in self.colors.items():
            d = [(v[i] - self.bg[i]) / self.bg[i] for i in range(4)]
            n2 = sum(x * x for x in d)
            if n2 < 0.01:
                raise CalibrationError(f"El color '{name}' es indistinguible del fondo")
            self._dirs[name] = (d, n2)

    def analyse(self, reading):
        """Devuelve (mejor_color o None, cobertura 0..1, residuo relativo)."""
        x = [(v - self.bg[i]) / self.bg[i] for i, v in enumerate(_vec(reading))]
        best = (None, 0.0, 1e9)
        for name, (d, n2) in self._dirs.items():
            f = sum(x[i] * d[i] for i in range(4)) / n2
            f = min(1.0, max(0.0, f))
            res = sum((x[i] - f * d[i]) ** 2 for i in range(4)) ** 0.5 / (n2 ** 0.5)
            if f > 0.15 and res < best[2]:
                best = (name, f, res)
        if best[0] is None:
            # fondo: cobertura máxima entre colores (≈0)
            cov = max((min(1.0, max(0.0, sum(x[i] * d[i] for i in range(4)) / n2))
                       for d, n2 in self._dirs.values()), default=0.0)
            return None, cov, 0.0
        return best

    def save(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump({"background": self.bg, "colors": self.colors,
                       "date": time.strftime("%Y-%m-%d %H:%M:%S")}, f, indent=2)

    @classmethod
    def load(cls, path):
        if not os.path.exists(path):
            raise CalibrationError(f"No existe {path}. Ejecutad primero --calibrate.")
        with open(path) as f:
            d = json.load(f)
        return cls(d["background"], d["colors"])


class ColorLineFollower:
    def __init__(self, model, cfg=None, imu_feedback=True, events=None):
        self.cfg = cfg or _config
        self.model = model
        self.events = events or (lambda t: None)
        self.machine = LineFollowStateMachine(self.cfg, imu_feedback=imu_feedback, events=self.events)
        self.current = None
        self._candidate, self._count = None, 0

    def __call__(self, ctx):
        r = ctx.reading
        if r is None:
            return Command(0, 0, States.STOPPED, stop_reason="sin lectura del sensor de color")
        name, cov, res = self.model.analyse(r)
        fresh = getattr(r, "fresh", True)
        # Color confirmado tras 3 lecturas nuevas consecutivas con cobertura >= 0.6
        if fresh:
            if name is not None and cov >= 0.6 and res <= self.cfg.COLOR_MAX_RESIDUAL:
                if name == self._candidate:
                    self._count += 1
                else:
                    self._candidate, self._count = name, 1
                if self._count >= 3 and name != self.current:
                    self.current = name
                    self.events(f"color detectado: {name}")
            else:
                self._candidate, self._count = None, 0
        action = self.cfg.COLOR_ACTIONS.get(self.current, {}) if self.current else {}
        if action.get("stop"):
            return Command(0, 0, States.STOPPED, stop_reason=f"color '{self.current}': parada configurada")
        self.machine.speed_scale = float(action.get("speed_scale", 1.0))
        cmd = self.machine.step(cov, ctx, fresh=fresh)
        cmd.log["note"] = f"color={name} cur={self.current} res={res:.2f}"
        return cmd


def calibrate(sensor, cfg, samples=40):
    def collect(label):
        input(f"Colocad el sensor sobre {label} y pulsad Enter...")
        acc, n = [0.0] * 4, 0
        while n < samples:
            r = sensor.read()
            if r.fresh:
                if r.saturated:
                    print("  AVISO: lectura saturada; reducid COLOR_GAIN")
                acc = [a + v for a, v in zip(acc, _vec(r))]
                n += 1
            time.sleep(0.005)
        v = [a / n for a in acc]
        print(f"  {label}: C={v[0]:.0f} R={v[1]:.0f} G={v[2]:.0f} B={v[3]:.0f}")
        return v

    bg = collect("el FONDO")
    colors = {name: collect(f"el color '{name}'") for name in cfg.COLOR_NAMES}
    return ColorModel(bg, colors)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--calibrate", action="store_true")
    ap.add_argument("--no-motors", action="store_true")
    ap.add_argument("--no-imu", action="store_true")
    ap.add_argument("--perfil", choices=["suave", "rapido"], default="suave",
                    help="suave: menos zigzag; rapido: ~30%% más velocidad con más oscilación")
    ap.add_argument("--no-distance", action="store_true")
    ap.add_argument("--edge", choices=["left", "right"])
    ap.add_argument("--max-time", type=float)
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)
    ov = dict(_config.PROFILES.get(args.perfil, {}))
    if args.edge:
        ov["EDGE_SIDE"] = args.edge
    if args.no_imu:
        ov["IMU_MODEL"] = "none"
    if args.no_motors:
        ov["STALL_DETECTION"] = False
    if args.no_distance:
        ov["DISTANCE_ENABLED"] = False
    cfg = _config.as_namespace(**ov)
    try:
        rt = build_runtime("color_line", cfg, console=not args.quiet, motors=not args.no_motors,
                           need_distance=not args.calibrate)
    except Exception as e:
        print(f"No se pudo iniciar: {e}")
        return 1
    try:
        if args.calibrate:
            model = calibrate(rt.color, cfg)
            model.save(cfg.COLOR_CALIBRATION_FILE)
            print(f"Guardado en {cfg.COLOR_CALIBRATION_FILE}")
            return 0
        model = ColorModel.load(cfg.COLOR_CALIBRATION_FILE)
        print("Calibrando IMU: no mováis el rover...")
        rt.calibrate_imu()
        rt.reset_yaw()
        follower = ColorLineFollower(model, cfg, imu_feedback=rt.imu is not None, events=rt.logger.event)
        rt.on_imu_disabled = lambda: setattr(follower.machine.mixer, "imu_feedback", False)
        step = follower
        if rt.distance is not None:
            step = ObstacleGuard(follower, cfg, events=rt.logger.event, on_resume=follower.machine.resume)
        print(f"Fin: {rt.run(step, max_time=args.max_time, drive_enabled=not args.no_motors)}")
        return 0
    except (CalibrationError, SafetyStop) as e:
        print(f"ERROR: {e}")
        return 1
    finally:
        rt.close()


if __name__ == "__main__":
    sys.exit(main())
