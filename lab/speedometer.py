import math
import time

import config
import encoder_client


class Speedometer:
    def __init__(self):
        self.last_a, self.last_b = encoder_client.counts()
        self.origin_a, self.origin_b = self.last_a, self.last_b
        self.last_t = time.monotonic()

    def sample(self):
        now = time.monotonic()
        a, b = encoder_client.counts()
        dt = now - self.last_t
        da, db = a - self.last_a, b - self.last_b
        self.last_a, self.last_b, self.last_t = a, b, now

        if dt <= 0:
            return 0.0, 0.0

        a_rpm = (da / config.PULSES_PER_REV_A) / dt * 60.0
        b_rpm = (db / config.PULSES_PER_REV_B) / dt * 60.0
        return a_rpm, b_rpm

    def distance_cm(self):
        a, b = encoder_client.counts()
        circumference = math.pi * config.WHEEL_DIAMETER_CM
        return (
            (a - self.origin_a) / config.PULSES_PER_REV_A * circumference,
            (b - self.origin_b) / config.PULSES_PER_REV_B * circumference,
        )
