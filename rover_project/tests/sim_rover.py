"""
sim_rover.py — Simulador cinemático SOLO PARA PRUEBAS AUTOMÁTICAS (no se usa en el rover).

Permite ejecutar el bucle real (drive_loop.RoverRuntime) y los seguidores reales
sin hardware, sustituyendo encodered, IMU, sensor de color y motores por modelos:

  * Pista: segmentos rectos y arcos; línea negra de ancho configurable.
  * Sensor: zona de medida circular adelantada respecto al eje; lectura "clear"
    = mezcla lineal fondo/negro según la fracción cubierta + ruido.
  * Motores: primer orden con ganancia distinta en A y B y zona muerta
    (reproduce "mismo PWM, distinta velocidad").
  * Encoders: cuentas enteras; opcionalmente canal 2 muerto (sin signo).
  * IMU: velocidad angular real + bias + ruido.

Los resultados del simulador NO demuestran que el rover real funcione; sirven
para detectar errores de lógica (signos, estados, búsquedas infinitas...).
"""

import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lab"))

import config  # noqa: E402
from color_sensor import Reading  # noqa: E402
from encoder_client import EncoderError  # noqa: E402
from rover import Rover, side_values  # noqa: E402


# ---------------------------------------------------------------------------
# Pista
# ---------------------------------------------------------------------------
class _Seg:
    pass


class _Line(_Seg):
    def __init__(self, x, y, h, length, s0):
        self.x, self.y, self.h, self.L, self.s0 = x, y, h, length, s0
        self.ux, self.uy = math.cos(h), math.sin(h)

    def end(self):
        return self.x + self.ux * self.L, self.y + self.uy * self.L, self.h

    def nearest(self, px, py):
        t = (px - self.x) * self.ux + (py - self.y) * self.uy
        t = max(0.0, min(self.L, t))
        qx, qy = self.x + self.ux * t, self.y + self.uy * t
        return math.hypot(px - qx, py - qy), self.s0 + t


class _Arc(_Seg):
    def __init__(self, x, y, h, radius, sweep, s0):
        self.R, self.sweep, self.s0 = radius, sweep, s0
        side = 1.0 if sweep > 0 else -1.0  # + = izquierda
        self.cx = x - side * radius * math.sin(h)
        self.cy = y + side * radius * math.cos(h)
        self.a0 = math.atan2(y - self.cy, x - self.cx)
        self.h0 = h
        self.L = abs(sweep) * radius

    def end(self):
        a = self.a0 + self.sweep
        return self.cx + self.R * math.cos(a), self.cy + self.R * math.sin(a), self.h0 + self.sweep

    def nearest(self, px, py):
        sg = 1.0 if self.sweep > 0 else -1.0
        a = math.atan2(py - self.cy, px - self.cx)
        rel = ((a - self.a0) * sg) % (2 * math.pi)
        if rel <= abs(self.sweep):
            return abs(math.hypot(px - self.cx, py - self.cy) - self.R), self.s0 + rel * self.R
        best = None
        for r in (0.0, abs(self.sweep)):
            ang = self.a0 + sg * r
            d = math.hypot(px - (self.cx + self.R * math.cos(ang)), py - (self.cy + self.R * math.sin(ang)))
            if best is None or d < best[0]:
                best = (d, self.s0 + r * self.R)
        return best


def _seg_point(seg, t):
    """Punto a distancia t (cm) desde el inicio del segmento."""
    if isinstance(seg, _Line):
        return seg.x + seg.ux * t, seg.y + seg.uy * t
    sg = 1.0 if seg.sweep > 0 else -1.0
    a = seg.a0 + sg * t / seg.R
    return seg.cx + seg.R * math.cos(a), seg.cy + seg.R * math.sin(a)


class Track:
    def __init__(self, width_cm=1.9, x=0.0, y=0.0, heading=0.0):
        self.width = width_cm
        self.segs = []
        self._x, self._y, self._h = x, y, heading
        self.length = 0.0

    def straight(self, length):
        s = _Line(self._x, self._y, self._h, length, self.length)
        self._add(s)
        return self

    def arc(self, radius, degrees):
        s = _Arc(self._x, self._y, self._h, radius, math.radians(degrees), self.length)
        self._add(s)
        return self

    def _add(self, s):
        self.segs.append(s)
        self.length += s.L
        self._x, self._y, self._h = s.end()

    def nearest(self, px, py):
        best = (1e9, 0.0)
        for s in self.segs:
            d = s.nearest(px, py)
            if d[0] < best[0]:
                best = d
        return best

    def point_at(self, s_cm):
        s_cm = max(0.0, min(self.length, s_cm))
        for seg in self.segs:
            if s_cm <= seg.s0 + seg.L:
                return _seg_point(seg, s_cm - seg.s0)
        return _seg_point(self.segs[-1], self.segs[-1].L)

    def is_black(self, px, py):
        return self.nearest(px, py)[0] <= self.width / 2


# ---------------------------------------------------------------------------
# Mundo físico
# ---------------------------------------------------------------------------
class SimWorld:
    def __init__(self, track, cfg, start_x=2.0, edge="left", seed=1, motor_gain_a=110.0, motor_gain_b=125.0,
                 deadzone=0.08, tau=0.08, sensor_offset=6.0, footprint_r=0.35, white=1200.0, black=250.0,
                 noise=8.0, gyro_bias=0.6, gyro_noise=0.3, dead_ch2=False, start_heading_deg=0.0,
                 start_lateral=None, motor_curves=None, load_factor=1.0, delay_s=0.0,
                 breakaway=None):
        self.track, self.cfg = track, cfg
        # motor_curves: {"a": [(pwm, rpm), ...], "b": [...]} medidas reales (curva no lineal + zona muerta)
        self.motor_curves = motor_curves
        self.load_factor = load_factor
        self.delay_s = delay_s          # retardo puro orden->motor (latencias, holguras)
        self.breakaway = breakaway or {}  # PWM mínimo para arrancar desde parado con carga
        self._duty_hist = []
        self.rng = random.Random(seed)
        self.t = 0.0
        self.L = sensor_offset
        h = math.radians(start_heading_deg)
        lateral = (track.width / 2 if edge == "left" else -track.width / 2) if start_lateral is None else start_lateral
        sx, sy = start_x, lateral
        self.x, self.y, self.th = sx - self.L * math.cos(h), sy - self.L * math.sin(h), h
        self.gain = {"a": motor_gain_a, "b": motor_gain_b}
        self.deadzone, self.tau = deadzone, tau
        self.rpm = {"a": 0.0, "b": 0.0}
        self.counts = {"a": 0.0, "b": 0.0}
        self.dead_ch2 = dead_ch2
        self.fp_r = footprint_r
        self.white, self.black, self.noise = white, black, noise
        self.gyro_bias, self.gyro_noise = gyro_bias, gyro_noise
        self.omega_true = 0.0
        self.rover = None
        self.motor_dead = {"a": False, "b": False}
        self.trace = []
        self._pts = [(0, 0)] + [(footprint_r * k * math.cos(a), footprint_r * k * math.sin(a))
                                for k in (0.33, 0.66, 1.0) for a in [i * math.pi / 8 for i in range(16)]]

    def sensor_pos(self):
        return self.x + self.L * math.cos(self.th), self.y + self.L * math.sin(self.th)

    def coverage(self):
        sx, sy = self.sensor_pos()
        n = sum(self.track.is_black(sx + dx, sy + dy) for dx, dy in self._pts)
        return n / len(self._pts)

    def _curve_rpm(self, key, duty):
        pts = self.motor_curves[key]
        d = abs(duty)
        if d < pts[0][0]:
            return 0.0
        for (d1, r1), (d2, r2) in zip(pts, pts[1:]):
            if d <= d2:
                return (r1 + (r2 - r1) * (d - d1) / (d2 - d1)) * self.load_factor
        return pts[-1][1] * self.load_factor

    def _motor(self, key, duty, dt):
        if self.motor_curves:
            target = math.copysign(self._curve_rpm(key, duty), duty)
        else:
            mag = max(0.0, abs(duty) - self.deadzone) / (1 - self.deadzone)
            target = math.copysign(self.gain[key] * mag, duty)
        if self.motor_dead[key]:
            target = 0.0
        if abs(self.rpm[key]) < 1.0 and abs(duty) < self.breakaway.get(key, 0.0):
            target = 0.0  # rozamiento estático: no arranca
        self.rpm[key] += (target - self.rpm[key]) * (dt / self.tau)
        ppr = self.cfg.PULSES_PER_REV_A if key == "a" else self.cfg.PULSES_PER_REV_B
        inc = self.rpm[key] / 60.0 * ppr * dt
        self.counts[key] += abs(inc) if self.dead_ch2 else inc

    def advance_to(self, t_end, h=0.002):
        while self.t < t_end - 1e-12:
            dt = min(h, t_end - self.t)
            da, db = self.rover.duty_a, self.rover.duty_b
            if self.delay_s > 0:
                self._duty_hist.append((self.t, da, db))
                while len(self._duty_hist) > 1 and self._duty_hist[1][0] <= self.t - self.delay_s:
                    self._duty_hist.pop(0)
                _, da, db = self._duty_hist[0]
            self._motor("a", da, dt)
            self._motor("b", db, dt)
            rl, rr = side_values(self.cfg, self.rpm["a"], self.rpm["b"])
            k = math.pi * self.cfg.WHEEL_DIAMETER_CM / 60.0
            vl, vr = rl * k, rr * k
            v = (vl + vr) / 2
            w = (vr - vl) / self.cfg.TRACK_WIDTH_CM
            self.omega_true = math.degrees(w)
            self.x += v * math.cos(self.th) * dt
            self.y += v * math.sin(self.th) * dt
            self.th += w * dt
            self.t += dt
        sx, sy = self.sensor_pos()
        d, s = self.track.nearest(sx, sy)
        self.trace.append((self.t, d, s, self.coverage()))


class SimClock:
    def __init__(self, world):
        self.w = world

    def now(self):
        return self.w.t

    def sleep_until(self, t):
        self.w.advance_to(t)


class SimEncoders:
    def __init__(self, world, fail_after_s=None):
        self.w, self.fail_after_s = world, fail_after_s

    def get_counts(self):
        if self.fail_after_s is not None and self.w.t >= self.fail_after_s:
            raise EncoderError("timeout simulado")
        return int(self.w.counts["a"]), int(self.w.counts["b"])

    def close(self):
        pass


class SimIMU:
    name = "SimIMU"

    def __init__(self, world):
        self.w = world
        self.bias = 0.0
        self.rate = 0.0

    def calibrate(self, *a, **k):
        self.bias = self.w.gyro_bias
        return self.bias, self.w.gyro_noise

    def read_rate(self):
        raw = self.w.omega_true + self.w.gyro_bias + self.w.rng.gauss(0, self.w.gyro_noise)
        self.rate = raw - self.bias
        return self.rate

    def close(self):
        pass


class SimColor:
    def __init__(self, world):
        self.w = world

    def read(self):
        cov = self.w.coverage()
        c = self.w.white - (self.w.white - self.w.black) * cov + self.w.rng.gauss(0, self.w.noise)
        c = int(max(0, c))
        return Reading(c, c // 3, c // 3, c // 3, self.w.t, True, False)

    def close(self):
        pass


class SimDistance:
    """Sensor IR simulado: objeto puntual en (x, y) presente entre t_on y t_off.
    Tensión alta (2.3 V) si está delante a menos de range_cm; baja (0.8 V) si no."""

    def __init__(self, world, x, y, t_on=0.0, t_off=1e9, range_cm=20.0, fail_after_s=None):
        self.w, self.x, self.y = world, x, y
        self.t_on, self.t_off, self.range = t_on, t_off, range_cm
        self.fail_after_s = fail_after_s

    def read(self):
        if self.fail_after_s is not None and self.w.t >= self.fail_after_s:
            from distance_sensor import DistanceSensorError
            raise DistanceSensorError("sensor desconectado (simulado)")
        if not (self.t_on <= self.w.t < self.t_off):
            return 0.8
        sx, sy = self.w.sensor_pos()
        dx, dy = self.x - sx, self.y - sy
        ahead = dx * math.cos(self.w.th) + dy * math.sin(self.w.th)
        lateral = abs(-dx * math.sin(self.w.th) + dy * math.cos(self.w.th))
        return 2.3 if 0 < ahead <= self.range and lateral < 6.0 else 0.8

    def close(self):
        pass


class SimLeader:
    """Robot de delante: avanza por la línea según un perfil [(t_hasta, velocidad_cm_s), ...].
    s_rear = posición de su parte TRASERA a lo largo de la pista."""

    def __init__(self, track, s0, profile):
        self.track, self.s, self.profile = track, s0, profile
        self.t_last = 0.0

    def update(self, t):
        while self.t_last < t:
            dt = min(0.01, t - self.t_last)
            v = next((vv for tt, vv in self.profile if self.t_last < tt), 0.0)
            self.s = min(self.track.length, self.s + v * dt)
            self.t_last += dt

    def rear_xy(self):
        return self.track.point_at(self.s)


class SimConvoyDistance:
    """Sensor IR: haz estrecho (±cone_deg) que mide la distancia a la primera superficie del
    líder, modelado como un rectángulo (ancho x largo) orientado según la pista.
    Modelo de tensión V ≈ 51.75/d (2,3 V a 22,5 cm, como medisteis)."""

    def __init__(self, world, leader, k=51.75, cone_deg=8.0, max_cm=70.0, width=12.0, length=15.0):
        self.w, self.leader, self.k = world, leader, k
        self.cone, self.max_cm = math.radians(cone_deg), max_cm
        self.width, self.length = width, length
        self.gaps = []

    def _leader_box(self):
        tr, s = self.leader.track, self.leader.s
        x0, y0 = tr.point_at(s)
        x1, y1 = tr.point_at(min(tr.length, s + 1.0))
        if (x1, y1) == (x0, y0):
            x1, y1 = tr.point_at(s)
            xm, ym = tr.point_at(max(0.0, s - 1.0))
            h = math.atan2(y0 - ym, x0 - xm)
        else:
            h = math.atan2(y1 - y0, x1 - x0)
        ux, uy = math.cos(h), math.sin(h)
        px, py = -uy * self.width / 2, ux * self.width / 2
        rl, rr = (x0 + px, y0 + py), (x0 - px, y0 - py)
        fl = (rl[0] + ux * self.length, rl[1] + uy * self.length)
        fr = (rr[0] + ux * self.length, rr[1] + uy * self.length)
        return [(rl, rr), (rr, fr), (fr, fl), (fl, rl)]

    @staticmethod
    def _ray_seg(ox, oy, dx, dy, a, b):
        ex, ey = b[0] - a[0], b[1] - a[1]
        den = dx * ey - dy * ex
        if abs(den) < 1e-9:
            return None
        t = ((a[0] - ox) * ey - (a[1] - oy) * ex) / den
        u = ((a[0] - ox) * dy - (a[1] - oy) * dx) / den
        return t if t > 0 and 0 <= u <= 1 else None

    def measure(self):
        sx, sy = self.w.sensor_pos()
        box = self._leader_box()
        best = None
        for k in range(-4, 5):
            ang = self.w.th + self.cone * k / 4
            dx, dy = math.cos(ang), math.sin(ang)
            for a, b in box:
                t = self._ray_seg(sx, sy, dx, dy, a, b)
                if t is not None and (best is None or t < best):
                    best = t
        return best

    def true_gap(self):
        """Distancia mínima real del sensor a la caja del líder (para comprobar colisiones)."""
        sx, sy = self.w.sensor_pos()
        best = 1e9
        for a, b in self._leader_box():
            ex, ey = b[0] - a[0], b[1] - a[1]
            u = max(0.0, min(1.0, ((sx - a[0]) * ex + (sy - a[1]) * ey) / (ex * ex + ey * ey)))
            best = min(best, math.hypot(a[0] + u * ex - sx, a[1] + u * ey - sy))
        return best

    def read(self):
        self.leader.update(self.w.t)
        self.gaps.append((self.w.t, self.true_gap()))
        d = self.measure()
        if d is None or d > self.max_cm:
            return 0.4                     # nada delante
        return min(3.0, self.k / max(d, 1.0))

    def close(self):
        pass


def convoy_test_calibration():
    from distance_sensor import DistanceCalibration
    return DistanceCalibration([(d, 51.75 / d) for d in (15, 20, 25, 30, 35, 40, 50, 60)])


def make_runtime(world, cfg, imu=True, encoder_fail_after=None, distance=None):
    from drive_loop import RoverRuntime

    rover = Rover(cfg, backend="mock")
    world.rover = rover
    return RoverRuntime(rover, SimEncoders(world, encoder_fail_after), SimColor(world),
                        SimIMU(world) if imu else None, cfg, clock=SimClock(world), distance=distance)


def sim_config(**overrides):
    """Configuración para simulación: FF coherentes con los motores simulados."""
    base = dict(FF_A_FORWARD_PER_RPM=1 / 110 * 0.92, FF_B_FORWARD_PER_RPM=1 / 125 * 0.92,
                FF_A_BACKWARD_PER_RPM=1 / 110 * 0.92, FF_B_BACKWARD_PER_RPM=1 / 125 * 0.92,
                FF_A_STATIC=0.08, FF_B_STATIC=0.08, KP_A=0.004, KI_A=0.02, KP_B=0.004, KI_B=0.02,
                MAX_RPM=90.0, TRACK_WIDTH_CM=13.0, RPM_PER_DEGPS=None, MOTOR_A_SIDE="left",
                IMU_MODEL="none")
    base.update(overrides)
    return config.as_namespace(**base)


# Curvas medidas en el rover NikiC (29-10, pidtest --ff-fit, ruedas en el aire): (PWM, RPM)
REAL_CURVES = {
    "a": [(0.25, 0.0), (0.3, 23.3), (0.35, 27.7), (0.4, 31.7), (0.5, 39.5), (0.6, 46.7), (0.7, 53.1),
          (0.8, 59.0), (0.9, 64.5), (1.0, 71.4)],
    "b": [(0.075, 0.0), (0.1, 24.7), (0.125, 47.7), (0.15, 76.8), (0.175, 95.2), (0.2, 117.0), (0.25, 143.8),
          (0.3, 162.0), (0.35, 176.4), (0.4, 186.4), (0.5, 200.0), (0.6, 210.1), (0.7, 215.2), (0.8, 220.4),
          (0.9, 225.3), (1.0, 231.2)],
}


def real_rover_config(**overrides):
    """Configuración del rover real (config.py + config_local medido el 29-10)."""
    base = dict(
        FF_A_TABLE=[(r, d) for d, r in REAL_CURVES["a"] if r > 0],
        FF_B_TABLE=[(r, d) for d, r in REAL_CURVES["b"] if r > 0],
        KP_A=0.005, KI_A=0.01, IMU_MODEL="none",
    )
    base.update(overrides)
    return config.as_namespace(**base)
