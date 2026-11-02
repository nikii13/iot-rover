"""
rover.py — Interfaz de motores del rover. ÚNICO módulo que toca los GPIO de motor.

    from rover import Rover
    with Rover() as r:
        r.set_motors(0.4, 0.4)   # PWM -1..1 para motor A y motor B
        r.stop()

Funciones públicas (se conservan alias habituales para scripts antiguos):
    set_motors(a, b)         PWM firmado de ambos motores
    set_motor_a(v) / set_motor_b(v)
    forward(speed) / backward(speed) / left(speed) / right(speed)
    stop()                   ambos motores a 0 (rueda libre)
    close()                  stop + liberar GPIO

Los ajustes de polaridad (MOTOR_*_INVERT) y el límite DUTY_LIMIT se aplican aquí,
de modo que ninguna capa superior puede enviar más PWM del permitido.
"""

import config as _config


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class _MockMotor:
    """Motor simulado: guarda el último valor (pruebas sin hardware)."""

    def __init__(self, *_, **__):
        self.value = 0.0

    def close(self):
        self.value = 0.0


class _GpiozeroMotor:
    def __init__(self, pin_fwd, pin_back, freq):
        from gpiozero import Motor  # importación diferida: solo en la Pi

        self._m = Motor(forward=pin_fwd, backward=pin_back, pwm=True)
        try:
            self._m.forward_device.frequency = freq
            self._m.backward_device.frequency = freq
        except Exception:
            pass  # algunos backends no permiten cambiar la frecuencia

    @property
    def value(self):
        return self._m.value

    @value.setter
    def value(self, v):
        self._m.value = v

    def close(self):
        try:
            self._m.stop()
        finally:
            self._m.close()


class Rover:
    def __init__(self, cfg=None, backend=None):
        self.cfg = cfg or _config
        backend = backend or self.cfg.MOTOR_BACKEND
        if backend == "mock":
            factory = lambda p1, p2: _MockMotor()  # noqa: E731
        elif backend == "gpiozero":
            factory = lambda p1, p2: _GpiozeroMotor(p1, p2, self.cfg.MOTOR_PWM_FREQUENCY_HZ)  # noqa: E731
        else:
            raise ValueError(f"MOTOR_BACKEND desconocido: {backend}")
        self._a = factory(self.cfg.MOTOR_A_PIN_1, self.cfg.MOTOR_A_PIN_2)
        try:
            self._b = factory(self.cfg.MOTOR_B_PIN_1, self.cfg.MOTOR_B_PIN_2)
        except Exception:
            self._a.close()
            raise
        self.duty_a = 0.0
        self.duty_b = 0.0
        self._closed = False

    # -- control -----------------------------------------------------------
    def set_motors(self, duty_a, duty_b):
        """Aplica PWM firmado (-1..1) a ambos motores, con límite y polaridad."""
        if self._closed:
            return
        lim = float(self.cfg.DUTY_LIMIT)
        self.duty_a = _clamp(float(duty_a), -lim, lim)
        self.duty_b = _clamp(float(duty_b), -lim, lim)
        self._a.value = -self.duty_a if self.cfg.MOTOR_A_INVERT else self.duty_a
        self._b.value = -self.duty_b if self.cfg.MOTOR_B_INVERT else self.duty_b

    def set_motor_a(self, duty):
        self.set_motors(duty, self.duty_b)

    def set_motor_b(self, duty):
        self.set_motors(self.duty_a, duty)

    def set_left_right(self, duty_left, duty_right):
        """PWM por lado físico (usa MOTOR_A_SIDE)."""
        if self.cfg.MOTOR_A_SIDE == "left":
            self.set_motors(duty_left, duty_right)
        else:
            self.set_motors(duty_right, duty_left)

    def forward(self, speed=0.5):
        self.set_motors(abs(speed), abs(speed))

    def backward(self, speed=0.5):
        self.set_motors(-abs(speed), -abs(speed))

    def left(self, speed=0.5):
        self.set_left_right(-abs(speed), abs(speed))

    def right(self, speed=0.5):
        self.set_left_right(abs(speed), -abs(speed))

    def stop(self):
        if not self._closed:
            self.set_motors(0.0, 0.0)

    def close(self):
        if self._closed:
            return
        try:
            self.stop()
        finally:
            self._closed = True
            for m in (self._a, self._b):
                try:
                    m.close()
                except Exception:
                    pass

    # -- context manager ---------------------------------------------------
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


def side_values(cfg, value_a, value_b):
    """Convierte valores (A, B) a (izquierda, derecha)."""
    return (value_a, value_b) if cfg.MOTOR_A_SIDE == "left" else (value_b, value_a)


def ab_values(cfg, left, right):
    """Convierte valores (izquierda, derecha) a (A, B)."""
    return (left, right) if cfg.MOTOR_A_SIDE == "left" else (right, left)
