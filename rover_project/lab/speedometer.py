#!/usr/bin/env python3
"""
speedometer.py — Convierte cuentas de encoder en RPM de rueda.

    rpm = (Δcuentas / PULSES_PER_REV) / Δt[s] * 60

- Δt con reloj monotónico (lo aporta quien llama).
- Rechaza saltos imposibles (> MAX_PLAUSIBLE_RPM) como lectura inválida.
- Con ENCODER_SIGN_FROM_COMMAND, el sentido se toma de la orden aplicada al
  motor (necesario mientras el canal 2 de los encoders no sea fiable).

Ejecutado directamente muestra las RPM en vivo (girad las ruedas a mano o
moved el rover con otro programa).
"""

import math
import time

import config as _config


class WheelSpeed:
    def __init__(self, pulses_per_rev, alpha, max_plausible_rpm, sign_from_command):
        self.ppr = float(pulses_per_rev)
        self.alpha = float(alpha)
        self.max_rpm = float(max_plausible_rpm)
        self.sign_from_command = sign_from_command
        self.last_count = None
        self.rpm_raw = 0.0
        self.rpm = 0.0
        self.invalid = 0
        self._last_dir = 1.0

    def reset(self, count=None):
        self.last_count = count
        self.rpm_raw = self.rpm = 0.0

    def update(self, count, dt, command_sign=0.0):
        """Devuelve True si la lectura es válida."""
        if command_sign:
            self._last_dir = 1.0 if command_sign > 0 else -1.0
        if self.last_count is None or dt <= 0:
            self.last_count = count
            return True
        delta = count - self.last_count
        self.last_count = count
        rpm = delta / self.ppr / dt * 60.0
        if not math.isfinite(rpm) or abs(rpm) > self.max_rpm:
            self.invalid += 1
            return False
        if self.sign_from_command:
            rpm = abs(rpm) * self._last_dir
        self.rpm_raw = rpm
        self.rpm = self.alpha * rpm + (1.0 - self.alpha) * self.rpm
        return True


class Speedometer:
    """RPM de los motores A y B a partir de las cuentas de encodered."""

    def __init__(self, cfg=None):
        cfg = cfg or _config
        self.cfg = cfg
        mk = lambda ppr: WheelSpeed(ppr, cfg.RPM_FILTER_ALPHA, cfg.MAX_PLAUSIBLE_RPM,  # noqa: E731
                                    cfg.ENCODER_SIGN_FROM_COMMAND)
        self.a = mk(cfg.PULSES_PER_REV_A)
        self.b = mk(cfg.PULSES_PER_REV_B)

    def update(self, counts, dt, command_sign_a=0.0, command_sign_b=0.0):
        ok_a = self.a.update(counts[0], dt, command_sign_a)
        ok_b = self.b.update(counts[1], dt, command_sign_b)
        return ok_a and ok_b

    @property
    def rpm_a(self):
        return self.a.rpm

    @property
    def rpm_b(self):
        return self.b.rpm

    def reset(self, counts=None):
        self.a.reset(None if counts is None else counts[0])
        self.b.reset(None if counts is None else counts[1])


def rpm_to_cm_s(rpm, cfg=None):
    cfg = cfg or _config
    return rpm * math.pi * cfg.WHEEL_DIAMETER_CM / 60.0


def main():
    from encoder_client import EncoderClient, EncoderError

    cfg = _config.as_namespace(ENCODER_SIGN_FROM_COMMAND=False)  # a mano: el sentido viene del encoder
    sp = Speedometer(cfg)
    with EncoderClient() as enc:
        t_prev = time.monotonic()
        try:
            while True:
                time.sleep(0.2)
                t = time.monotonic()
                try:
                    counts = enc.get_counts()
                except EncoderError as e:
                    print(f"ERROR: {e}")
                    return 1
                sp.update(counts, t - t_prev)
                t_prev = t
                print(f"cuentas A={counts[0]:>8} B={counts[1]:>8} | RPM A={sp.rpm_a:7.1f} B={sp.rpm_b:7.1f} "
                      f"| cm/s A={rpm_to_cm_s(sp.rpm_a):6.1f} B={rpm_to_cm_s(sp.rpm_b):6.1f}", flush=True)
        except KeyboardInterrupt:
            return 0


if __name__ == "__main__":
    raise SystemExit(main())
