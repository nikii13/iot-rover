"""
line_control.py — Piezas comunes del seguimiento de línea (negra y de colores).

Restricción física de partida: UN solo sensor de color bajo el rover. No mide
la posición lateral de la línea, pero su lectura varía de forma continua
cuando su zona de medida cruza el BORDE entre línea y fondo. Por eso:

  * Se sigue un borde (EDGE_SIDE), no el centro de la línea. Sobre el borde la
    "cobertura" (0 = fondo, 1 = línea) dice hacia qué lado corregir:
        borde izquierdo: demasiado oscuro -> girar a la izquierda
                         demasiado claro  -> girar a la derecha
    Funciona igual para curvas a izquierda y a derecha; no depende del sentido
    de la pista ni de que la primera curva sea hacia un lado concreto.
  * Si el sensor queda en fondo claro un tiempo (LINE_LOST), el lado de búsqueda
    se decide con el último sentido de giro registrado y, si no lo hay, con el
    lado del borde seguido. La búsqueda es acotada en ángulo y tiempo.

Jerarquía de control (una sola autoridad sobre el PWM, ver drive_loop.py):
    Nivel A  EdgeFollower      cobertura -> v (RPM) y omega_ref (deg/s)
    Nivel B  DriveMixer        omega_ref + IMU -> RPM izquierda/derecha
             La IMU regula la VELOCIDAD ANGULAR hacia omega_ref (lo que pide
             la línea), nunca un rumbo fijo: no puede oponerse a una curva.
    Nivel C  speed_control     RPM objetivo -> PWM (PID con encoders)
"""

import collections
import json
import math
import os
import time

import config as _config
from drive_loop import Command


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


# ---------------------------------------------------------------------------
# Calibración de superficie
# ---------------------------------------------------------------------------
class CalibrationError(RuntimeError):
    pass


class SurfaceCalibration:
    """Valores del sensor sobre línea y fondo. cobertura = (fondo - v) / (fondo - línea)."""

    def __init__(self, line_value, background_value, channel="clear", source="manual"):
        self.line = float(line_value)
        self.background = float(background_value)
        self.channel = channel
        self.source = source

    @classmethod
    def from_samples(cls, line_samples, background_samples, channel="clear", source="manual"):
        med = lambda xs: sorted(xs)[len(xs) // 2]  # noqa: E731
        if not line_samples or not background_samples:
            raise CalibrationError("No hay muestras de calibración")
        return cls(med(line_samples), med(background_samples), channel, source)

    @classmethod
    def from_sweep(cls, values, channel="clear"):
        """Calibración automática: el sensor pasó sobre fondo y línea (percentiles 5 y 95)."""
        if len(values) < 20:
            raise CalibrationError("Muestras insuficientes en el barrido de calibración")
        s = sorted(values)
        return cls(s[int(0.05 * len(s))], s[int(0.95 * len(s)) - 1], channel, "auto")

    def contrast(self):
        return (self.background - self.line) / self.background if self.background > 0 else 0.0

    def validate(self, cfg=None):
        cfg = cfg or _config
        if self.background <= self.line:
            raise CalibrationError(
                f"El fondo ({self.background:.0f}) no es más claro que la línea ({self.line:.0f}). "
                "¿Muestras intercambiadas o sensor mal orientado?")
        if self.contrast() < cfg.MIN_CALIBRATION_CONTRAST:
            raise CalibrationError(
                f"Contraste insuficiente ({self.contrast()*100:.0f} %, mínimo {cfg.MIN_CALIBRATION_CONTRAST*100:.0f} %). "
                "Revisad altura del sensor, ganancia o iluminación.")
        return self

    def coverage(self, reading):
        v = reading.value(self.channel) if hasattr(reading, "value") else float(reading)
        return _clamp((self.background - v) / (self.background - self.line), 0.0, 1.0)

    def save(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump({"line": self.line, "background": self.background, "channel": self.channel,
                       "source": self.source, "date": time.strftime("%Y-%m-%d %H:%M:%S")}, f, indent=2)

    @classmethod
    def load(cls, path):
        if not os.path.exists(path):
            raise CalibrationError(f"No existe {path}. Ejecutad primero la calibración (--calibrate).")
        with open(path) as f:
            d = json.load(f)
        return cls(d["line"], d["background"], d.get("channel", "clear"), d.get("source", "file"))

    def __repr__(self):
        return (f"SurfaceCalibration(línea={self.line:.0f}, fondo={self.background:.0f}, "
                f"canal={self.channel}, contraste={self.contrast()*100:.0f}%)")


class SurfaceClassifier:
    """Clasifica la cobertura filtrada en LINE / EDGE / BACKGROUND con histéresis."""

    LINE, EDGE, BACKGROUND = "LINE", "EDGE", "BACKGROUND"

    def __init__(self, cfg=None):
        cfg = cfg or _config
        self.cfg = cfg
        self.state = self.EDGE

    def update(self, cov):
        c = self.cfg
        if self.state == self.LINE:
            if cov < c.BLACK_EXIT:
                self.state = self.BACKGROUND if cov <= c.WHITE_ENTER else self.EDGE
        elif self.state == self.BACKGROUND:
            if cov > c.WHITE_EXIT:
                self.state = self.LINE if cov >= c.BLACK_ENTER else self.EDGE
        else:
            if cov >= c.BLACK_ENTER:
                self.state = self.LINE
            elif cov <= c.WHITE_ENTER:
                self.state = self.BACKGROUND
        return self.state


# ---------------------------------------------------------------------------
# Nivel A: seguimiento de borde
# ---------------------------------------------------------------------------
class EdgeFollower:
    def __init__(self, cfg=None):
        cfg = cfg or _config
        self.cfg = cfg
        if cfg.EDGE_SIDE not in ("left", "right"):
            raise ValueError("EDGE_SIDE debe ser 'left' o 'right'")
        # + = girar a la izquierda. Borde izquierdo: exceso de negro -> izquierda.
        self.edge_sign = 1.0 if cfg.EDGE_SIDE == "left" else -1.0
        self.reset()

    def reset(self):
        self.prev_cov = None
        self.dcov = 0.0
        self.integral = 0.0
        self.v_f = None

    def update(self, cov, dt, fresh=True, speed_scale=1.0):
        """Devuelve (v_rpm, omega_ref_dps, error)."""
        c = self.cfg
        e = cov - c.EDGE_SETPOINT
        if fresh and self.prev_cov is not None and dt > 0:
            raw_d = (cov - self.prev_cov) / dt
            self.dcov = 0.5 * raw_d + 0.5 * self.dcov
        if fresh:
            self.prev_cov = cov
        # Integral: en una curva larga el P necesita un error permanente para girar, y con
        # el borde izquierdo eso deja el sensor cerca del blanco en las curvas a la derecha
        # (lado sin margen). El término integral devuelve el sensor al borde (cov≈0.5).
        ki = getattr(c, "LINE_KI_DPS", 0.0)
        lim = getattr(c, "LINE_I_LIMIT_DPS", 0.0)
        if ki and dt > 0 and abs(e) < 0.4:
            self.integral = _clamp(self.integral + ki * e * dt, -lim, lim)
        omega = self.edge_sign * (c.LINE_KP_DPS * e + c.LINE_KD_DPS * self.dcov + self.integral)
        omega = _clamp(omega, -c.OMEGA_MAX_DPS, c.OMEGA_MAX_DPS)
        v = c.BASE_RPM * speed_scale * max(0.0, 1.0 - c.SLOWDOWN_GAIN * abs(e))
        v /= 1.0 + abs(omega) / c.TURN_SLOWDOWN_DPS
        v = max(min(c.MIN_FOLLOW_RPM, c.BASE_RPM * speed_scale), v)
        # Suavizado de velocidad: frena rápido (entrada en curva) y acelera despacio, para que
        # cada pequeña corrección no provoque un acelerón/frenazo que alimente el vaivén.
        up, down = getattr(c, "SPEED_TAU_UP_S", 0.0), getattr(c, "SPEED_TAU_DOWN_S", 0.0)
        if (up > 0 or down > 0) and dt > 0:
            if self.v_f is None:
                self.v_f = v
            tau = up if v > self.v_f else down
            self.v_f += (v - self.v_f) * (min(1.0, dt / tau) if tau > 0 else 1.0)
            v = self.v_f
        return v, omega, e

    def default_search_direction(self):
        """En fondo claro la línea está al lado contrario del borde seguido (+1 izq, -1 der)."""
        return -self.edge_sign


# ---------------------------------------------------------------------------
# Nivel B: mezcla diferencial + estabilización con IMU
# ---------------------------------------------------------------------------
class DriveMixer:
    def __init__(self, cfg=None, imu_feedback=True):
        cfg = cfg or _config
        self.cfg = cfg
        self.k_diff = _config.rpm_per_degps(cfg)
        self.imu_feedback = imu_feedback
        self.last_correction = 0.0

    def mix(self, v_rpm, omega_ref, omega_meas=None, allow_reverse=None):
        """RPM (izquierda, derecha). omega + = izquierda => rueda derecha más rápida."""
        c = self.cfg
        allow_reverse = c.ALLOW_WHEEL_REVERSE if allow_reverse is None else allow_reverse
        diff = self.k_diff * omega_ref
        corr = 0.0
        if self.imu_feedback and omega_meas is not None:
            corr = _clamp(c.YAW_RATE_KP * (omega_ref - omega_meas),
                          -c.YAW_RATE_CORRECTION_LIMIT_RPM, c.YAW_RATE_CORRECTION_LIMIT_RPM)
        self.last_correction = corr
        diff += corr
        max_rpm = float(c.MAX_RPM)
        diff = _clamp(diff, -2 * max_rpm, 2 * max_rpm)
        # Si una rueda satura, se sacrifica velocidad de avance, no el giro.
        v = v_rpm
        if abs(v) + abs(diff) / 2 > max_rpm:
            v = math.copysign(max(0.0, max_rpm - abs(diff) / 2), v_rpm)
        left, right = v - diff / 2, v + diff / 2
        if not allow_reverse and v_rpm > 0:
            left, right = max(0.0, left), max(0.0, right)
        return left, right


# ---------------------------------------------------------------------------
# Máquina de estados del seguimiento
# ---------------------------------------------------------------------------
class States:
    CALIBRATING = "CALIBRATING"
    FOLLOWING = "FOLLOWING"
    CORRECTING_LEFT = "CORRECTING_LEFT"
    CORRECTING_RIGHT = "CORRECTING_RIGHT"
    LINE_LOST = "LINE_LOST"
    SEARCHING = "SEARCHING"
    STOPPED = "STOPPED"


class LineFollowStateMachine:
    """
    FOLLOWING / CORRECTING_LEFT / CORRECTING_RIGHT
        controlador de borde continuo; el estado indica hacia dónde corrige.
        -> LINE_LOST si fondo claro durante LOST_CONFIRM_S
        -> STOPPED si línea saturada (negro total) durante BLACK_SATURATED_MAX_S
    LINE_LOST
        frena LOST_BRAKE_S y elige lado de búsqueda (último giro o lado del borde)
    SEARCHING
        giro sobre sí mismo: barrido 1 hacia el lado elegido, barrido 2 hacia el
        otro, vuelta al rumbo inicial. Línea encontrada -> FOLLOWING.
        Sin éxito o SEARCH_TIMEOUT_S -> STOPPED.
    """

    def __init__(self, cfg=None, imu_feedback=True, events=None):
        cfg = cfg or _config
        self.cfg = cfg
        self.follower = EdgeFollower(cfg)
        self.mixer = DriveMixer(cfg, imu_feedback)
        self.classifier = SurfaceClassifier(cfg)
        self.events = events or (lambda text: None)
        self.state = None
        self.cov_f = None
        self.speed_scale = 1.0
        self._history = collections.deque()
        self._since = None
        self._background_since = None
        self._background_yaw = 0.0
        self._line_since = None
        self.search_dir = 0.0
        self.search_phase = 0
        self.search_start_yaw = 0.0
        self.search_start_t = 0.0
        self.lost_count = 0
        self.stop_reason = None
        self._loss_times = collections.deque()

    # -- utilidades --------------------------------------------------------
    def _set(self, state, t, reason=None):
        if state != self.state:
            msg = f"{self.state or 'INICIO'} -> {state}"
            if reason:
                msg += f" ({reason})"
            self.events(msg)
        self.state = state
        self._since = t

    def _turn_memory(self, t, omega):
        self._history.append((t, omega))
        while self._history and t - self._history[0][0] > self.cfg.LAST_TURN_WINDOW_S:
            self._history.popleft()

    def _avg_recent_turn(self):
        if not self._history:
            return 0.0
        return sum(w for _, w in self._history) / len(self._history)

    def _cmd(self, left, right, extra):
        return Command(left, right, self.state, extra)

    # -- paso principal ----------------------------------------------------
    def step(self, coverage, ctx, fresh=True):
        c = self.cfg
        t = ctx.t
        a = c.DARKNESS_FILTER_ALPHA
        self.cov_f = coverage if self.cov_f is None else a * coverage + (1 - a) * self.cov_f
        surface = self.classifier.update(self.cov_f)
        log = {"coverage": self.cov_f, "surface": surface}

        if self.state is None:
            if surface == SurfaceClassifier.BACKGROUND:
                self._start_search(ctx, self.follower.default_search_direction(), "línea no detectada al arrancar")
            else:
                self._set(States.FOLLOWING, t)

        if self.state in (States.FOLLOWING, States.CORRECTING_LEFT, States.CORRECTING_RIGHT):
            # Pérdida de línea
            if surface == SurfaceClassifier.BACKGROUND:
                # En fondo claro el controlador sigue girando al máximo hacia la línea (es
                # una salida normal del borde). Solo se declara perdida si, tras LOST_CONFIRM_S
                # o tras girar LOST_TURN_DEG hacia ella, sigue sin aparecer.
                if self._background_since is None:
                    self._background_since = t
                    self._background_yaw = ctx.yaw_deg
                elif (t - self._background_since >= c.LOST_CONFIRM_S
                      or abs(ctx.yaw_deg - self._background_yaw) >= c.LOST_TURN_DEG):
                    avg = self._avg_recent_turn()
                    if abs(avg) >= c.LAST_TURN_MIN_DPS:
                        d, why = math.copysign(1.0, avg), f"último giro medio {avg:+.0f} deg/s"
                    else:
                        d, why = self.follower.default_search_direction(), f"lado del borde {c.EDGE_SIDE}"
                    self.search_dir = d
                    self.lost_count += 1
                    self._loss_times.append(t)
                    while self._loss_times and t - self._loss_times[0] > c.LOSS_WINDOW_S:
                        self._loss_times.popleft()
                    if len(self._loss_times) >= c.MAX_LOSSES:
                        return self._stop(t, log, f"{len(self._loss_times)} pérdidas de línea en "
                                                  f"{c.LOSS_WINDOW_S:.0f}s: el seguimiento no es estable "
                                                  "(revisad calibración, LINE_KP_DPS o velocidad)")
                    self._set(States.LINE_LOST, t,
                              f"fondo claro durante {t - self._background_since:.2f}s y "
                              f"{abs(ctx.yaw_deg - self._background_yaw):.0f}° de giro; buscar hacia "
                              f"{'IZQUIERDA' if d > 0 else 'DERECHA'} por {why}")
                    self.follower.reset()
                    log["note"] = "line_lost"
                    return self._cmd(0.0, 0.0, log)
            else:
                self._background_since = None
            # Negro saturado demasiado tiempo
            if surface == SurfaceClassifier.LINE:
                if self._line_since is None:
                    self._line_since = t
                elif t - self._line_since >= c.BLACK_SATURATED_MAX_S:
                    return self._stop(t, log, f"línea saturada (negro total) durante {c.BLACK_SATURATED_MAX_S:.1f}s: "
                                              "zona negra, fin de pista o rover perpendicular a la línea")
            else:
                self._line_since = None

            v, omega_ref, err = self.follower.update(self.cov_f, ctx.dt, fresh, self.speed_scale)
            self._turn_memory(t, omega_ref)
            if abs(err) < c.CORRECTING_THRESHOLD:
                label = States.FOLLOWING
            else:
                label = States.CORRECTING_LEFT if omega_ref > 0 else States.CORRECTING_RIGHT
            if label != self.state:
                self.state = label  # sin evento: cambia muchas veces por segundo
            left, right = self.mixer.mix(v, omega_ref, ctx.omega_meas)
            log.update({"omega_ref": omega_ref, "yaw_corr_rpm": self.mixer.last_correction,
                        "v_rpm": v, "line_err": err})
            return self._cmd(left, right, log)

        if self.state == States.LINE_LOST:
            if t - self._since >= c.LOST_BRAKE_S:
                self._start_search(ctx, self.search_dir, None)
            else:
                return self._cmd(0.0, 0.0, log)

        if self.state == States.SEARCHING:
            return self._search_step(ctx, log)

        if self.state == States.STOPPED:
            cmd = self._cmd(0.0, 0.0, log)
            cmd.stop_reason = self.stop_reason
            return cmd

        return self._cmd(0.0, 0.0, log)

    def resume(self):
        """Tras una pausa (obstáculo): olvida temporizadores y memoria de giro para no
        interpretar el tiempo parado como línea perdida o negro continuo."""
        self.follower.reset()
        self._background_since = None
        self._line_since = None
        self._history.clear()
        if self.state == States.SEARCHING:
            self.search_start_t = None  # se reinicia en el siguiente paso

    def _start_search(self, ctx, direction, reason):
        self.search_dir = direction
        self.search_phase = 1
        self.search_start_yaw = ctx.yaw_deg
        self.search_start_t = ctx.t
        self.search_wheel_turn = 0.0
        self._set(States.SEARCHING, ctx.t, reason or
                  f"barrido 1 hacia {'IZQUIERDA' if direction > 0 else 'DERECHA'} hasta {self.cfg.SEARCH_FIRST_SWEEP_DEG:.0f}°")

    def _search_step(self, ctx, log):
        c = self.cfg
        rel = ctx.yaw_deg - self.search_start_yaw  # + = izquierda
        log["note"] = f"search_phase={self.search_phase} rel_yaw={rel:+.0f}"
        if self.cov_f >= c.SEARCH_FOUND_COVERAGE:
            self.follower.reset()
            self._background_since = None
            self._line_since = None
            self._history.clear()
            self._set(States.FOLLOWING, ctx.t, f"línea encontrada tras girar {rel:+.0f}°")
            return self._cmd(0.0, 0.0, log)
        if self.search_start_t is None:
            self.search_start_t = ctx.t
        if ctx.t - self.search_start_t > c.SEARCH_TIMEOUT_S:
            return self._stop(ctx.t, log, f"búsqueda sin éxito en {c.SEARCH_TIMEOUT_S:.1f}s")
        # Límite independiente de la IMU: giro acumulado según las ruedas (por si la IMU falla)
        self.search_wheel_turn += abs(ctx.rpm_right - ctx.rpm_left) / self.mixer.k_diff * ctx.dt
        max_turn = c.SEARCH_FIRST_SWEEP_DEG + 2 * c.SEARCH_SECOND_SWEEP_DEG + 120.0
        if self.search_wheel_turn > max_turn:
            return self._stop(ctx.t, log, f"búsqueda: {self.search_wheel_turn:.0f}° girados según las ruedas "
                                          "(¿IMU_YAW_SIGN invertido?)")

        d = self.search_dir
        if self.search_phase == 1:
            target, sweep_dir = d * c.SEARCH_FIRST_SWEEP_DEG, d
            if d * rel >= c.SEARCH_FIRST_SWEEP_DEG:
                self.search_phase = 2
                self.events(f"SEARCHING: barrido 2 hacia {'IZQUIERDA' if -d > 0 else 'DERECHA'} "
                            f"hasta {c.SEARCH_SECOND_SWEEP_DEG:.0f}°")
        if self.search_phase == 2:
            target, sweep_dir = -d * c.SEARCH_SECOND_SWEEP_DEG, -d
            if -d * rel >= c.SEARCH_SECOND_SWEEP_DEG:
                self.search_phase = 3
                self.events("SEARCHING: volviendo al rumbo inicial")
        if self.search_phase == 3:
            target, sweep_dir = 0.0, d
            if d * rel >= 0.0:
                return self._stop(ctx.t, log, "línea no encontrada tras barrer ambos lados")
        log["note"] += f" target={target:+.0f}"
        omega_ref = sweep_dir * c.SEARCH_OMEGA_DPS
        left, right = self.mixer.mix(0.0, omega_ref, ctx.omega_meas, allow_reverse=True)
        log.update({"omega_ref": omega_ref, "yaw_corr_rpm": self.mixer.last_correction, "v_rpm": 0.0})
        return self._cmd(left, right, log)

    def _stop(self, t, log, reason):
        self.stop_reason = reason
        self._set(States.STOPPED, t, reason)
        cmd = self._cmd(0.0, 0.0, log)
        cmd.stop_reason = reason
        return cmd


# ---------------------------------------------------------------------------
# Parada por obstáculo (sensor de distancia)
# ---------------------------------------------------------------------------
class ObstacleGuard:
    """
    Envuelve un paso de seguimiento. Si el rover avanza y el sensor de distancia
    supera OBSTACLE_VOLTAGE, se detiene (estado OBSTACLE) y se bloquea hasta que la
    tensión esté por debajo de CLEAR_VOLTAGE durante OBSTACLE_CLEAR_S (misma lógica
    que rc_pid_lab9_working.py). Entonces reanuda (OBSTACLE_RESUME) o termina.
    Los giros sobre sí mismo (búsqueda) no se bloquean, como en vuestro programa.
    """

    def __init__(self, step, cfg=None, events=None, on_resume=None):
        self.step = step
        self.cfg = cfg or _config
        self.events = events or (lambda t: None)
        self.on_resume = on_resume
        self.latched = False
        self.since = 0.0
        self.clear_s = 0.0
        self.count = 0

    def __call__(self, ctx):
        c = self.cfg
        v = ctx.distance_v
        if self.latched:
            if v is not None and v < c.CLEAR_VOLTAGE:
                self.clear_s += ctx.dt
            else:
                self.clear_s = 0.0
            if self.clear_s >= c.OBSTACLE_CLEAR_S:
                self.latched = False
                self.events(f"Camino despejado ({v:.2f} V durante {self.clear_s:.1f}s): se reanuda el seguimiento")
                if self.on_resume:
                    self.on_resume()
            else:
                cmd = Command(0.0, 0.0, "OBSTACLE", {"distance_v": v, "note": f"obstaculo clear_s={self.clear_s:.1f}"})
                if c.OBSTACLE_MAX_WAIT_S and ctx.t - self.since > c.OBSTACLE_MAX_WAIT_S:
                    cmd.stop_reason = f"obstáculo durante más de {c.OBSTACLE_MAX_WAIT_S:.0f}s"
                return cmd
        cmd = self.step(ctx)
        if cmd.stop_reason:
            return cmd
        forward = (cmd.target_left + cmd.target_right) / 2.0 > 0.0
        if v is not None and forward and v >= c.OBSTACLE_VOLTAGE:
            self.latched = True
            self.since = ctx.t
            self.clear_s = 0.0
            self.count += 1
            self.events(f"OBSTÁCULO a ~20 cm ({v:.2f} V >= {c.OBSTACLE_VOLTAGE:.2f} V): rover detenido")
            stop = Command(0.0, 0.0, "OBSTACLE", {"distance_v": v, "note": "obstaculo detectado"})
            if not c.OBSTACLE_RESUME:
                stop.stop_reason = f"obstáculo detectado ({v:.2f} V)"
            return stop
        if v is not None:
            cmd.log["distance_v"] = v
        return cmd


# ---------------------------------------------------------------------------
# Convoy: seguir la línea manteniendo la separación con el robot de delante
# ---------------------------------------------------------------------------
class ConvoyController:
    """
    Regula la VELOCIDAD de avance del seguidor de línea con la distancia al robot de delante
    medida por el sensor IR (calibrada a cm). La dirección la sigue decidiendo el seguidor.

    Líder a la vista (lectura real):
        e = d - CONVOY_TARGET_CM;  v = KP*e + I   (cm/s, limitado a [0, CONVOY_MAX_RPM])
        d < CONVOY_STOP_CM  -> parada inmediata
    Líder fuera del haz tras haberlo visto (típico en curvas, el sensor apunta recto):
        se supone PARADO donde se vio: d_est = última distancia - lo que avanzamos (encoders);
        se acerca hasta TARGET-CONVOY_MEMORY_APPROACH_CM solo con P (rápido si está lejos), para
        girar con la curva y volver a verlo. Si espera parado más de CONVOY_MEMORY_S sin verlo,
        supone que el líder se fue y avanza a CONVOY_BLIND_RPM hasta encontrarlo.
    Nunca visto: avanza a CONVOY_BLIND_RPM.

    Las ruedas no giran estables por debajo de ~20 RPM: histéresis de parada/arranque
    (CONVOY_STOP_RPM / CONVOY_START_RPM). Mientras espera, la máquina de estados no avanza
    (no cuenta como línea perdida) y al arrancar se llama a machine.resume().
    """

    WAIT, MOVING, TOO_CLOSE = "CONVOY_WAIT", "CONVOY_MOVING", "CONVOY_TOO_CLOSE"

    def __init__(self, step, machine, calibration, cfg=None, events=None):
        self.step = step
        self.machine = machine
        self.cal = calibration
        self.cfg = cfg or _config
        self.events = events or (lambda t: None)
        self.cms_per_rpm = math.pi * self.cfg.WHEEL_DIAMETER_CM / 60.0
        self.mode = self.WAIT
        self.d_f = None
        self.integral = 0.0
        self.v_rpm = 0.0
        self.memory_cm = None       # distancia estimada al líder cuando no se ve
        self.wait_unseen_s = 0.0
        self.source = "none"        # "sensor" | "estimada" | "libre"

    def _measure(self, volts, dt, travel_cm):
        c = self.cfg
        d = self.cal.cm(volts)
        if d is not None:
            a = c.CONVOY_DIST_ALPHA
            # acercarse se detecta sin filtrar (seguridad); alejarse se filtra
            self.d_f = d if (self.d_f is None or self.source != "sensor" or d < self.d_f) \
                else a * d + (1 - a) * self.d_f
            self.memory_cm = self.d_f
            self.wait_unseen_s = 0.0
            self.source = "sensor"
            return self.d_f
        if self.memory_cm is not None:
            self.memory_cm = max(0.0, self.memory_cm - travel_cm)
            if self.mode != self.MOVING:
                self.wait_unseen_s += dt
            if self.wait_unseen_s <= c.CONVOY_MEMORY_S:
                if self.source == "sensor":
                    self.events(f"CONVOY: líder fuera del haz; supongo que está parado a ~{self.memory_cm:.0f} cm")
                self.source = "estimada"
                return self.memory_cm
            self.events(f"CONVOY: {c.CONVOY_MEMORY_S:.0f}s sin ver al líder: supongo que se ha ido, avanzo despacio")
            self.memory_cm = None
        self.source = "libre"
        return None

    def __call__(self, ctx):
        c = self.cfg
        if ctx.distance_v is None:
            return Command(0.0, 0.0, "STOPPED", stop_reason="convoy sin sensor de distancia")
        travel = max(0.0, (ctx.rpm_left + ctx.rpm_right) / 2.0 * self.cms_per_rpm * ctx.dt)
        d = self._measure(ctx.distance_v, ctx.dt, travel)
        vmax_cms = c.CONVOY_MAX_RPM * self.cms_per_rpm
        blind_cms = c.CONVOY_BLIND_RPM * self.cms_per_rpm

        if self.source == "sensor":
            e = d - c.CONVOY_TARGET_CM
            if ctx.dt > 0:
                gain = c.CONVOY_KI * (1.0 if e > 0 else 4.0)       # se vacía rápido si está cerca
                if abs(e) < 15.0 or e < 0:
                    self.integral = _clamp(self.integral + gain * e * ctx.dt, 0.0, 0.7 * vmax_cms)
            v_cms = _clamp(c.CONVOY_KP * e + self.integral, 0.0, vmax_cms)
        elif self.source == "estimada":
            # Estimación pesimista (líder supuesto parado): la distancia real nunca es menor.
            # Lejos se puede ir rápido; al acercarse la P frena hasta TARGET-APPROACH.
            self.integral = 0.0
            target = max(c.CONVOY_STOP_CM + 2.0, c.CONVOY_TARGET_CM - c.CONVOY_MEMORY_APPROACH_CM)
            v_cms = _clamp(c.CONVOY_KP * (d - target), 0.0, vmax_cms)
        else:
            self.integral = 0.0
            v_cms = blind_cms
        v_rpm = v_cms / self.cms_per_rpm
        self.v_rpm = v_rpm
        dtxt = f"{d:.1f}" if d is not None else "-"
        log = {"distance_v": ctx.distance_v,
               "note": f"convoy d={dtxt}cm ({self.source}) v={v_rpm:.1f}rpm I={self.integral:.2f}"}

        if self.mode == self.MOVING and self.machine.state in (States.LINE_LOST, States.SEARCHING):
            # La búsqueda gira sobre sí misma (no avanza): se deja terminar sin interrumpirla.
            cmd = self.step(ctx)
            cmd.log.update(log)
            return cmd

        if self.source == "sensor" and d < c.CONVOY_STOP_CM:
            if self.mode != self.TOO_CLOSE:
                self.events(f"CONVOY: demasiado cerca ({d:.0f} cm < {c.CONVOY_STOP_CM:.0f}): parada")
            self.mode = self.TOO_CLOSE
            self.integral = 0.0
            return Command(0.0, 0.0, self.TOO_CLOSE, log)

        if self.mode != self.MOVING:
            if v_rpm >= c.CONVOY_START_RPM:
                self.events(f"CONVOY: en marcha (líder a {dtxt} cm, {self.source})")
                self.mode = self.MOVING
                self.machine.resume()
            else:
                self.mode = self.WAIT
                return Command(0.0, 0.0, self.WAIT, log)
        elif v_rpm < c.CONVOY_STOP_RPM:
            self.events(f"CONVOY: el de delante se ha parado ({dtxt} cm, {self.source}) -> espero")
            self.mode = self.WAIT
            return Command(0.0, 0.0, self.WAIT, log)

        scale = v_rpm / c.BASE_RPM
        if self.machine.state in (States.CORRECTING_LEFT, States.CORRECTING_RIGHT):
            scale = min(scale, 1.0)   # ir más rápido que la base solo en tramos bien centrados
        self.machine.speed_scale = scale
        cmd = self.step(ctx)
        cmd.log.update(log)
        return cmd
