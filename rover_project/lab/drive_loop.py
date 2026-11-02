"""
drive_loop.py — Único bucle de control del rover (usado por todos los programas de conducción).

Orden de cada ciclo (CONTROL_LOOP_HZ):
    1. encodered (socket)  -> cuentas -> RPM medidas (speedometer)
    2. IMU                 -> velocidad angular medida (o odometría si no hay IMU)
    3. sensor de color     -> lectura
    4. step(ctx)           -> RPM objetivo izquierda/derecha  (nivel A + B: trayectoria + IMU)
    5. WheelSpeedController-> PWM                              (nivel C: PID por motor)
    6. Rover.set_motors    -> única escritura de PWM del ciclo
    7. log no bloqueante

Seguridad: Ctrl+C / SIGTERM, fallos de encoder/IMU/sensor, motor bloqueado y
excepciones inesperadas detienen los motores (finally) y liberan recursos.

El hardware se inyecta, de modo que el mismo bucle se ejecuta en las pruebas
con un simulador (tests/sim_rover.py).
"""

import signal
import threading
import time
from dataclasses import dataclass, field

import config as _config
from encoder_client import EncoderError
from rover import ab_values, side_values
from speed_control import WheelSpeedController
from speedometer import Speedometer


class SafetyStop(RuntimeError):
    """Parada controlada con motivo legible."""


class MonotonicClock:
    def now(self):
        return time.monotonic()

    def sleep_until(self, t):
        d = t - time.monotonic()
        if d > 0:
            time.sleep(d)


@dataclass
class Context:
    t: float = 0.0               # segundos desde el inicio del bucle
    dt: float = 0.0
    reading: object = None       # color_sensor.Reading o None
    rpm_left: float = 0.0
    rpm_right: float = 0.0
    omega_meas: float = 0.0      # deg/s, + = izquierda
    omega_source: str = "none"   # "imu" | "encoders"
    distance_v: float = None     # tensión del sensor de distancia (None si no hay sensor)
    yaw_deg: float = 0.0         # integrado desde reset_yaw()
    cycle: int = 0


@dataclass
class Command:
    target_left: float = 0.0
    target_right: float = 0.0
    state: str = ""
    log: dict = field(default_factory=dict)
    stop_reason: str = None      # si no es None: parada controlada y fin del bucle


class RoverRuntime:
    def __init__(self, rover, encoders, color_sensor=None, imu=None, cfg=None, clock=None, logger=None,
                 distance=None):
        self.cfg = cfg or _config
        self.rover = rover
        self.encoders = encoders
        self.color = color_sensor
        self.imu = imu
        self.distance = distance
        self.clock = clock or MonotonicClock()
        self.logger = logger
        self.speedometer = Speedometer(self.cfg)
        self.pid = WheelSpeedController(self.cfg)
        self.rpm_per_degps = _config.rpm_per_degps(self.cfg)
        self.yaw_deg = 0.0
        self.stop_reason = None
        self._enc_failures = 0
        self._stall_since = [None, None]
        self._imu_agree = 0
        self.on_imu_disabled = None
        self._imu_disagree = 0

    # ------------------------------------------------------------------
    def check_services(self):
        """Comprueba que encodered responde antes de mover nada."""
        try:
            self.encoders.get_counts()
        except EncoderError as e:
            raise SafetyStop(f"Servicio de encoders no disponible: {e}") from e

    def calibrate_imu(self):
        if self.imu is not None:
            bias, std = self.imu.calibrate()
            self._event(f"IMU calibrada: bias={bias:.3f} deg/s ruido={std:.3f} deg/s")

    def reset_yaw(self):
        self.yaw_deg = 0.0

    def _event(self, text):
        if self.logger:
            self.logger.event(text)

    # ------------------------------------------------------------------
    def _read_encoders(self, dt):
        try:
            counts = self.encoders.get_counts()
            self._enc_failures = 0
        except EncoderError as e:
            self._enc_failures += 1
            if self._enc_failures >= self.cfg.ENCODER_MAX_CONSECUTIVE_FAILURES:
                raise SafetyStop(f"Encoders: {self._enc_failures} lecturas fallidas seguidas ({e})") from e
            return False
        ok = self.speedometer.update(counts, dt, self.rover.duty_a, self.rover.duty_b)
        if not ok:
            self._enc_failures += 1
            if self._enc_failures >= self.cfg.ENCODER_MAX_CONSECUTIVE_FAILURES:
                raise SafetyStop("Encoders: saltos de cuenta imposibles repetidos (lectura inválida)")
        return ok

    def _check_imu_sign(self, omega_imu, omega_enc):
        """Si la IMU indica sistemáticamente el giro contrario al de las ruedas,
        IMU_YAW_SIGN está mal: se desactiva la IMU (realimentaría al revés)."""
        if not getattr(self.cfg, "IMU_SIGN_CHECK", True):
            return
        if abs(omega_imu) < 25.0 or abs(omega_enc) < 25.0:
            return
        if (omega_imu > 0) == (omega_enc > 0):
            self._imu_agree += 1
        else:
            self._imu_disagree += 1
        n = self._imu_agree + self._imu_disagree
        if n >= 25 and self._imu_disagree > 0.7 * n:
            self._event("La IMU indica el giro CONTRARIO al de las ruedas: IMU_YAW_SIGN parece invertido. "
                        "Se desactiva la IMU en esta ejecución (poned IMU_YAW_SIGN = -1).")
            self.imu = None
            if hasattr(self, "on_imu_disabled") and self.on_imu_disabled:
                self.on_imu_disabled()

    def _check_stall(self, now):
        if not self.cfg.STALL_DETECTION:
            return
        for i, (duty, rpm, name) in enumerate(((self.rover.duty_a, self.speedometer.rpm_a, "A"),
                                               (self.rover.duty_b, self.speedometer.rpm_b, "B"))):
            if abs(duty) >= self.cfg.STALL_DUTY and abs(rpm) < self.cfg.STALL_RPM:
                if self._stall_since[i] is None:
                    self._stall_since[i] = now
                elif now - self._stall_since[i] >= self.cfg.STALL_TIME_S:
                    raise SafetyStop(
                        f"Motor {name}: PWM {duty:+.2f} durante {self.cfg.STALL_TIME_S:.1f}s sin pulsos de encoder "
                        f"(rueda bloqueada, motor sin alimentación o encoder {name} sin señal)")
            else:
                self._stall_since[i] = None

    # ------------------------------------------------------------------
    def run(self, step, max_time=None, drive_enabled=True):
        """Ejecuta el bucle hasta que step pida parar, max_time, Ctrl+C o fallo de seguridad."""
        period = 1.0 / self.cfg.CONTROL_LOOP_HZ
        prev_handler = None
        if threading.current_thread() is threading.main_thread():
            prev_handler = signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
        ctx = Context()
        t0 = self.clock.now()
        t_prev = t0
        next_tick = t0
        overruns = 0
        try:
            self.speedometer.reset()
            self._read_encoders(0.0)
            while True:
                next_tick += period
                now = self.clock.now()
                dt = now - t_prev
                t_prev = now
                ctx.t, ctx.dt = now - t0, dt
                ctx.cycle += 1

                self._read_encoders(dt)
                ctx.rpm_left, ctx.rpm_right = side_values(self.cfg, self.speedometer.rpm_a, self.speedometer.rpm_b)

                omega_enc = (ctx.rpm_right - ctx.rpm_left) / self.rpm_per_degps
                if self.imu is not None:
                    try:
                        ctx.omega_meas = self.imu.read_rate()
                        ctx.omega_source = "imu"
                        self._check_imu_sign(ctx.omega_meas, omega_enc)
                    except Exception as e:  # IMUError
                        if self.cfg.IMU_REQUIRED:
                            raise SafetyStop(f"IMU: {e}") from e
                        self._event(f"IMU perdida ({e}); se continúa con odometría de encoders")
                        self.imu = None
                if self.imu is None:
                    ctx.omega_meas = omega_enc
                    ctx.omega_source = "encoders"
                self.yaw_deg += ctx.omega_meas * dt
                ctx.yaw_deg = self.yaw_deg

                if self.color is not None:
                    try:
                        ctx.reading = self.color.read()
                    except Exception as e:  # ColorSensorError: sensor imprescindible
                        raise SafetyStop(f"Sensor de color: {e}") from e

                if self.distance is not None:
                    try:
                        ctx.distance_v = self.distance.read()
                    except Exception as e:  # DistanceSensorError
                        if getattr(self.cfg, "DISTANCE_REQUIRED", True):
                            raise SafetyStop(f"Sensor de distancia: {e}") from e
                        self._event(f"Sensor de distancia perdido ({e}); se continúa SIN detección de obstáculos")
                        self.distance = None
                        ctx.distance_v = None

                cmd = step(ctx)
                if cmd.stop_reason:
                    self.stop_reason = cmd.stop_reason
                    break
                if max_time is not None and ctx.t >= max_time:
                    self.stop_reason = f"tiempo máximo alcanzado ({max_time:.1f} s)"
                    break

                tgt_a, tgt_b = ab_values(self.cfg, cmd.target_left, cmd.target_right)
                duty_a, duty_b = self.pid.update(tgt_a, tgt_b, self.speedometer.rpm_a, self.speedometer.rpm_b, dt)
                if drive_enabled:
                    self.rover.set_motors(duty_a, duty_b)
                    self._check_stall(now)

                if self.logger:
                    row = {
                        "t": ctx.t, "state": cmd.state,
                        "target_rpm_a": self.pid.target_a, "target_rpm_b": self.pid.target_b,
                        "rpm_a": self.speedometer.rpm_a, "rpm_b": self.speedometer.rpm_b,
                        "err_a": self.pid.a.error, "err_b": self.pid.b.error,
                        "duty_a": duty_a, "duty_b": duty_b,
                        "omega_meas": ctx.omega_meas, "yaw_deg": ctx.yaw_deg,
                    }
                    if ctx.reading is not None:
                        row["raw"] = ctx.reading.c
                    if ctx.distance_v is not None:
                        row["distance_v"] = ctx.distance_v
                    row.update(cmd.log)
                    self.logger.log(row)

                if self.clock.now() > next_tick:
                    overruns += 1
                    next_tick = self.clock.now()  # no acumular retraso
                self.clock.sleep_until(next_tick)
        except KeyboardInterrupt:
            self.stop_reason = "interrumpido por el usuario (Ctrl+C / SIGTERM)"
        except SafetyStop as e:
            self.stop_reason = f"PARADA DE SEGURIDAD: {e}"
        except Exception as e:  # cualquier error inesperado: parar primero, informar después
            self.stop_reason = f"PARADA POR ERROR: {type(e).__name__}: {e}"
            raise
        finally:
            self.rover.stop()
            self.pid.reset()
            if prev_handler is not None:
                signal.signal(signal.SIGTERM, prev_handler)
            self._event(f"Motores detenidos. Motivo: {self.stop_reason}. Ciclos con retraso: {overruns}")
        return self.stop_reason

    def close(self):
        for res in (self.rover, self.color, self.imu, self.distance, self.encoders):
            if res is None:
                continue
            try:
                res.close()
            except Exception:
                pass
        if self.logger:
            self.logger.close()


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


def build_runtime(name, cfg=None, need_color=True, need_imu=None, console=True, motors=True, need_distance=False):
    """Crea el runtime con el hardware real. Si algo falla, libera lo ya abierto."""
    from color_sensor import ColorSensor
    from diag_log import DiagLogger
    from encoder_client import EncoderClient
    from imu import open_imu
    from rover import Rover

    cfg = cfg or _config
    opened = []
    try:
        enc = EncoderClient(cfg=cfg)
        opened.append(enc)
        rover = Rover(cfg, backend=None if motors else "mock")
        opened.append(rover)
        color = ColorSensor(cfg) if need_color else None
        if color:
            opened.append(color)
        imu = open_imu(cfg) if need_imu is not False else None
        if imu:
            opened.append(imu)
        distance = None
        if need_distance and getattr(cfg, "DISTANCE_ENABLED", False):
            from distance_sensor import DistanceSensor
            distance = DistanceSensor(cfg)
            opened.append(distance)
        logger = DiagLogger(name, cfg, console=console)
        opened.append(logger)
        rt = RoverRuntime(rover, enc, color, imu, cfg, logger=logger, distance=distance)
        rt.check_services()
        return rt
    except Exception:
        for r in reversed(opened):
            try:
                r.close()
            except Exception:
                pass
        raise
