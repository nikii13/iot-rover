from gpiozero import Motor

import config


class Rover:

    def __init__(self):

        self.motor_a = Motor(
            **config.MOTOR_A,
            pwm=True
        )

        self.motor_b = Motor(
            **config.MOTOR_B,
            pwm=True
        )

        self.stopped = False


    # ========================================================
    # LOW LEVEL MOTOR CONTROL
    # ========================================================

    @staticmethod
    def _set(motor, value):

        value = max(
            -1.0,
            min(1.0, float(value))
        )

        if value > 0:

            motor.forward(value)

        elif value < 0:

            motor.backward(-value)

        else:

            motor.stop()


    # ========================================================
    # WHEEL CONTROL
    # ========================================================

    def set_wheels(self, a, b):
        """
        Logical wheel commands.

        IMPORTANT:

        +A / +B = physical forward
        -A / -B = physical backward

        Motor orientation is corrected here so the rest of
        the software does not need to know how the motors are
        physically mounted.
        """

        if self.stopped:

            self.stop()
            return


        physical_a = (
            a * config.MOTOR_A_SIGN
        )

        physical_b = (
            b * config.MOTOR_B_SIGN
        )


        self._set(
            self.motor_a,
            physical_a
        )

        self._set(
            self.motor_b,
            physical_b
        )


    # ========================================================
    # HIGH LEVEL MOVEMENT
    # ========================================================

    def forward(self, speed=config.MANUAL_SPEED):
        """
        Move toward the physical FRONT of the rover.
        """

        self.set_wheels(
            speed,
            speed
        )


    def backward(self, speed=config.MANUAL_SPEED):
        """
        Move toward the physical REAR of the rover.
        """

        self.set_wheels(
            -speed,
            -speed
        )


    def left(self, speed=config.MANUAL_SPEED):
        """
        Forward curve toward physical LEFT.

        After redefining the physical front:
        A is the outer wheel
        B is the inner wheel.
        """

        self.set_wheels(
            speed,
            speed * 0.60
        )


    def right(self, speed=config.MANUAL_SPEED):
        """
        Forward curve toward physical RIGHT.
        """

        self.set_wheels(
            speed * 0.60,
            speed
        )


    def spin_left(self, speed=config.MANUAL_SPEED):

        self.set_wheels(
            -speed,
            speed
        )


    def spin_right(self, speed=config.MANUAL_SPEED):

        self.set_wheels(
            speed,
            -speed
        )


    # ========================================================
    # STOP
    # ========================================================

    def stop(self):

        self.motor_a.stop()
        self.motor_b.stop()


    def emergency_stop(self):

        self.stop()

        self.stopped = True


    def rearm(self):

        self.stop()

        self.stopped = False


    def close(self):

        self.stop()

        self.motor_a.close()
        self.motor_b.close()
