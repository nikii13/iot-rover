import config


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def sign(value):

    if value > 0:
        return 1.0

    if value < 0:
        return -1.0

    return 0.0


class WheelController:

    def __init__(self, wheel):

        self.wheel = wheel.upper()

        if self.wheel not in ("A", "B"):
            raise ValueError("Wheel must be A or B")

        self.integral = 0.0


    def reset(self):

        self.integral = 0.0


    def _parameters(self, target):

        if self.wheel == "A":

            kp = config.KP_A
            ki = config.KI_A

            ff = (
                config.FF_A_FORWARD_PER_RPM
                if target > 0
                else config.FF_A_BACKWARD_PER_RPM
            )

        else:

            kp = config.KP_B
            ki = config.KI_B

            ff = (
                config.FF_B_FORWARD_PER_RPM
                if target > 0
                else config.FF_B_BACKWARD_PER_RPM
            )

        return ff, kp, ki


    def update(self, target, measured_rpm, dt):

        if target == 0:

            self.reset()

            return 0.0


        target_rpm = abs(target)

        error = (
            target_rpm
            - measured_rpm
        )


        self.integral += error * dt

        self.integral = clamp(
            self.integral,
            -config.MAX_INTEGRAL,
            config.MAX_INTEGRAL
        )


        ff, kp, ki = self._parameters(
            target
        )


        duty = (
            target_rpm * ff
            + kp * error
            + ki * self.integral
        )


        duty = clamp(
            duty,
            0.0,
            1.0
        )


        return sign(target) * duty
