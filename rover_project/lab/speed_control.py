"""
speed_control.py — Control de velocidad de rueda (feed-forward + PI(D)) por motor.

Autoridad final sobre el PWM: el bucle principal llama una vez por ciclo a
WheelSpeedController.update(target_a, target_b, rpm_a, rpm_b, dt) y aplica el
resultado con Rover.set_motors(). Ningún otro módulo escribe PWM mientras
este controlador está activo. No hay hilos ni bucles propios.

Por motor:
    u = FF(r) + KP*e + I + KD*de/dt
        FF(r) = tabla medida FF_*_TABLE si existe (motores no lineales),
                si no FF_static*sign(r) + FF_per_rpm*r
    I += KI*e*dt   (solo si no satura en la misma dirección -> anti-windup)
    |I| <= INTEGRAL_LIMIT,  |u| <= DUTY_LIMIT
    r == 0  -> u = 0 y se reinicia el estado
    cambio de signo de r -> se reinicia el integrador
"""

import config as _config


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _sign(v):
    return (v > 0) - (v < 0)


def ff_from_table(table, rpm):
    """PWM de feed-forward interpolando una tabla medida [(rpm, pwm), ...] (rpm crecientes).
    Por debajo del primer punto devuelve el PWM de arranque; por encima, extrapola
    con la última pendiente. Se aplica al valor absoluto; el signo lo pone quien llama."""
    r = abs(rpm)
    pts = sorted(table)
    if r <= pts[0][0]:
        # Por debajo de la primera velocidad medida el motor sigue necesitando el PWM
        # de arranque (rozamiento estático): se usa ese mínimo para que arranque YA.
        return pts[0][1]
    for (r1, d1), (r2, d2) in zip(pts, pts[1:]):
        if r <= r2:
            return d1 + (d2 - d1) * (r - r1) / (r2 - r1) if r2 > r1 else d2
    (r1, d1), (r2, d2) = pts[-2], pts[-1]
    return d2 + (d2 - d1) * (r - r2) / (r2 - r1) if r2 > r1 else d2


class MotorPID:
    def __init__(self, kp, ki, kd, ff_fwd, ff_back, ff_static, integral_limit, duty_limit, ff_table=None,
                 min_rpm=0.0, kick_rate=0.0, kick_rpm=3.0, kick_delay=0.06, kick_max=0.6):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.ff_fwd, self.ff_back, self.ff_static = ff_fwd, ff_back, ff_static
        self.ff_table = ff_table
        # Por debajo de min_rpm la rueda no puede girar de forma estable (rozamiento estático):
        # se detiene en lugar de oscilar entre parada y ~20 RPM.
        self.min_rpm = min_rpm
        # Impulso de arranque: si la rueda debería girar pero sigue parada (rozamiento estático
        # mayor con carga, p. ej. al pivotar), se añade PWM que crece a kick_rate por segundo
        # hasta que se mueve; después se desvanece rápidamente.
        self.kick_rate, self.kick_rpm, self.kick_delay, self.kick_max = kick_rate, kick_rpm, kick_delay, kick_max
        self.integral_limit = integral_limit
        self.duty_limit = duty_limit
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.kick = 0.0
        self.stalled_s = 0.0
        self.prev_error = None
        self.last_target = 0.0
        self.error = 0.0
        self.output = 0.0
        self.ff = 0.0

    def update(self, target_rpm, measured_rpm, dt):
        if abs(target_rpm) < self.min_rpm:
            target_rpm = 0.0
        if target_rpm == 0.0:
            self.reset()
            return 0.0
        if _sign(target_rpm) != _sign(self.last_target):
            self.integral = 0.0
            self.prev_error = None
        self.last_target = target_rpm

        if self.ff_table:
            self.ff = _sign(target_rpm) * ff_from_table(self.ff_table, target_rpm)
        else:
            ff_gain = self.ff_fwd if target_rpm > 0 else self.ff_back
            self.ff = self.ff_static * _sign(target_rpm) + ff_gain * target_rpm
        e = target_rpm - measured_rpm
        self.error = e
        d = 0.0
        if self.kd and self.prev_error is not None and dt > 0:
            d = self.kd * (e - self.prev_error) / dt
        self.prev_error = e

        if self.kick_rate > 0 and dt > 0:
            if abs(measured_rpm) < self.kick_rpm:
                self.stalled_s += dt
                if self.stalled_s >= self.kick_delay:
                    self.kick = min(self.kick_max, self.kick + self.kick_rate * dt)
            else:
                self.stalled_s = 0.0
                self.kick *= 0.5
        d += _sign(target_rpm) * self.kick
        u_unsat = self.ff + self.kp * e + self.integral + d
        # Anti-windup por integración condicional
        saturating = abs(u_unsat) >= self.duty_limit and _sign(u_unsat) == _sign(e)
        if not saturating and dt > 0:
            self.integral = _clamp(self.integral + self.ki * e * dt, -self.integral_limit, self.integral_limit)
        u = self.ff + self.kp * e + self.integral + d
        # La señal no puede invertir el sentido pedido (evita "frenazos" con el integrador)
        if _sign(u) == -_sign(target_rpm):
            u = 0.0
        self.output = _clamp(u, -self.duty_limit, self.duty_limit)
        return self.output


class WheelSpeedController:
    def __init__(self, cfg=None):
        cfg = cfg or _config
        self.cfg = cfg
        lim = cfg.DUTY_LIMIT
        self.a = MotorPID(cfg.KP_A, cfg.KI_A, cfg.KD_A, cfg.FF_A_FORWARD_PER_RPM, cfg.FF_A_BACKWARD_PER_RPM,
                          cfg.FF_A_STATIC, cfg.INTEGRAL_LIMIT, lim, getattr(cfg, "FF_A_TABLE", None),
                          getattr(cfg, "MIN_WHEEL_RPM", 0.0), getattr(cfg, "STICTION_KICK_RATE", 0.0),
                          getattr(cfg, "STICTION_RPM", 3.0), getattr(cfg, "STICTION_DELAY_S", 0.06))
        self.b = MotorPID(cfg.KP_B, cfg.KI_B, cfg.KD_B, cfg.FF_B_FORWARD_PER_RPM, cfg.FF_B_BACKWARD_PER_RPM,
                          cfg.FF_B_STATIC, cfg.INTEGRAL_LIMIT, lim, getattr(cfg, "FF_B_TABLE", None),
                          getattr(cfg, "MIN_WHEEL_RPM", 0.0), getattr(cfg, "STICTION_KICK_RATE", 0.0),
                          getattr(cfg, "STICTION_RPM", 3.0), getattr(cfg, "STICTION_DELAY_S", 0.06))
        self.max_rpm = float(cfg.MAX_RPM)
        self.max_accel = float(getattr(cfg, "MAX_ACCEL_RPM_S", 0.0))
        self._mean = 0.0
        self.target_a = 0.0
        self.target_b = 0.0

    def update(self, target_a, target_b, rpm_a, rpm_b, dt):
        ta = _clamp(float(target_a), -self.max_rpm, self.max_rpm)
        tb = _clamp(float(target_b), -self.max_rpm, self.max_rpm)
        # Rampa SOLO sobre la componente común (avance): sincroniza el arranque de A
        # (PWM ~0.3) y B (~0.1) sin retrasar la diferencia entre ruedas (el giro).
        # Parar (ambos objetivos a 0) es inmediato.
        mean, diff = (ta + tb) / 2.0, (ta - tb) / 2.0
        if (ta == 0.0 and tb == 0.0) or self.max_accel <= 0 or dt <= 0:
            self._mean = mean
        else:
            step = self.max_accel * dt
            self._mean = _clamp(mean, self._mean - step, self._mean + step)
        self.target_a = _clamp(self._mean + diff, -self.max_rpm, self.max_rpm)
        self.target_b = _clamp(self._mean - diff, -self.max_rpm, self.max_rpm)
        return self.a.update(self.target_a, rpm_a, dt), self.b.update(self.target_b, rpm_b, dt)

    def reset(self):
        self.a.reset()
        self.b.reset()
        self.target_a = self.target_b = 0.0
        self._mean = 0.0
