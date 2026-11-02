#!/usr/bin/env python3

import curses
import time

from collections import deque
from statistics import median

from gpiozero import MCP3008

import encoder_client

from rover import Rover
from speedometer import Speedometer
from speed_control import WheelController


# ============================================================
# CONTROL LOOP
# ============================================================

DT = 0.10


# ============================================================
# MOVEMENT SPEEDS
# ============================================================

# Forward speed
FORWARD_RPM = 50.0


# Backward:
# reduced because it was too fast
BACKWARD_A_RPM = 35.0
BACKWARD_B_RPM = 35.0


# LEFT CURVE
#
# A = faster/outside wheel
# B = slower/inside wheel
LEFT_A_RPM = 50.0
LEFT_B_RPM = 30.0


# RIGHT CURVE
#
# D was turning less than A,
# therefore we make the difference larger.
RIGHT_A_RPM = 22.0
RIGHT_B_RPM = 50.0


# Rotation on the spot
SPIN_RPM = 35.0


# ============================================================
# DISTANCE SENSOR
# ============================================================

VREF = 3.3

# Around 20-25 cm we measured roughly 2.2-2.4 V
OBSTACLE_VOLTAGE = 2.0

# Used to determine that the obstacle has moved away
CLEAR_VOLTAGE = 1.7

# 10 control cycles ~= 1 second
CLEAR_SAMPLES_REQUIRED = 10


# ============================================================
# MAIN
# ============================================================

def main(stdscr):

    curses.curs_set(0)
    stdscr.nodelay(True)


    # ========================================================
    # SETUP - ROVER / ENCODERS
    # ========================================================

    encoder_client.reset()

    rover = Rover()

    speedometer = Speedometer()

    controller_a = WheelController("A")
    controller_b = WheelController("B")


    # ========================================================
    # SETUP - DISTANCE SENSOR
    # ========================================================

    distance_adc = MCP3008(
        channel=0,
        clock_pin=11,
        mosi_pin=10,
        miso_pin=9,
        select_pin=8
    )

    distance_samples = deque(maxlen=3)

    distance_voltage = 0.0


    # ========================================================
    # ROVER STATE
    # ========================================================

    target_a = 0.0
    target_b = 0.0

    command_a = 0.0
    command_b = 0.0

    mode = "STOP"


    # ========================================================
    # OBSTACLE STATE
    # ========================================================

    obstacle_latched = False

    clear_samples = 0


    last = time.monotonic()


    try:

        while True:

            # =================================================
            # READ DISTANCE SENSOR
            # =================================================

            raw = distance_adc.value

            distance_samples.append(raw)

            filtered_raw = median(
                distance_samples
            )

            distance_voltage = (
                filtered_raw * VREF
            )


            # =================================================
            # KEYBOARD
            # =================================================

            key = stdscr.getch()


            if key != -1:

                # Reset wheel controllers whenever
                # the driving command changes.

                controller_a.reset()
                controller_b.reset()


                # ------------------------------------------------
                # W - FORWARD
                # ------------------------------------------------

                if key in (
                    ord("w"),
                    ord("W")
                ):

                    if obstacle_latched:

                        target_a = 0.0
                        target_b = 0.0

                        rover.stop()

                        mode = "BLOCKED - OBSTACLE"

                    else:

                        target_a = FORWARD_RPM
                        target_b = FORWARD_RPM

                        mode = "FORWARD"


                # ------------------------------------------------
                # S - BACKWARD
                # ------------------------------------------------

                elif key in (
                    ord("s"),
                    ord("S")
                ):

                    # Backward is deliberately slower.
                    #
                    # It is also allowed after an obstacle stop
                    # so that the rover can move away.

                    target_a = -BACKWARD_A_RPM
                    target_b = -BACKWARD_B_RPM

                    mode = "BACKWARD"


                # ------------------------------------------------
                # A - LEFT CURVE
                # ------------------------------------------------

                elif key in (
                    ord("a"),
                    ord("A")
                ):

                    if obstacle_latched:

                        target_a = 0.0
                        target_b = 0.0

                        rover.stop()

                        mode = "BLOCKED - OBSTACLE"

                    else:

                        target_a = LEFT_A_RPM
                        target_b = LEFT_B_RPM

                        mode = "LEFT CURVE"


                # ------------------------------------------------
                # D - RIGHT CURVE
                # ------------------------------------------------

                elif key in (
                    ord("d"),
                    ord("D")
                ):

                    if obstacle_latched:

                        target_a = 0.0
                        target_b = 0.0

                        rover.stop()

                        mode = "BLOCKED - OBSTACLE"

                    else:

                        target_a = RIGHT_A_RPM
                        target_b = RIGHT_B_RPM

                        mode = "RIGHT CURVE"


                # ------------------------------------------------
                # Z - SPIN LEFT
                # ------------------------------------------------

                elif key in (
                    ord("z"),
                    ord("Z")
                ):

                    target_a = -SPIN_RPM
                    target_b = SPIN_RPM

                    mode = "SPIN LEFT"


                # ------------------------------------------------
                # C - SPIN RIGHT
                # ------------------------------------------------

                elif key in (
                    ord("c"),
                    ord("C")
                ):

                    target_a = SPIN_RPM
                    target_b = -SPIN_RPM

                    mode = "SPIN RIGHT"


                # ------------------------------------------------
                # STOP
                # ------------------------------------------------

                elif key in (
                    ord("e"),
                    ord("E"),
                    ord(" ")
                ):

                    target_a = 0.0
                    target_b = 0.0

                    command_a = 0.0
                    command_b = 0.0

                    controller_a.reset()
                    controller_b.reset()

                    rover.stop()

                    mode = "STOP"


                # ------------------------------------------------
                # MANUAL OBSTACLE REARM
                # ------------------------------------------------

                elif key in (
                    ord("r"),
                    ord("R")
                ):

                    if (
                        obstacle_latched
                        and
                        distance_voltage < CLEAR_VOLTAGE
                    ):

                        obstacle_latched = False

                        clear_samples = 0

                        target_a = 0.0
                        target_b = 0.0

                        rover.stop()

                        mode = "REARMED"

                    elif obstacle_latched:

                        mode = "OBSTACLE STILL DETECTED"


                # ------------------------------------------------
                # QUIT
                # ------------------------------------------------

                elif key in (
                    ord("q"),
                    ord("Q")
                ):

                    break


            # =================================================
            # COLLISION DETECTION
            # =================================================

            # Positive A and positive B means that the rover
            # is moving towards its physical front.

            moving_forward = (
                target_a > 0
                and
                target_b > 0
            )


            if (
                moving_forward
                and
                distance_voltage >= OBSTACLE_VOLTAGE
            ):

                obstacle_latched = True

                clear_samples = 0

                target_a = 0.0
                target_b = 0.0

                command_a = 0.0
                command_b = 0.0

                controller_a.reset()
                controller_b.reset()

                rover.stop()

                mode = "OBSTACLE STOP"


            # =================================================
            # CONTROL LOOP TIMING
            # =================================================

            now = time.monotonic()


            if (
                now - last < DT
            ):

                time.sleep(0.005)

                continue


            dt = now - last

            last = now


            # =================================================
            # AUTOMATIC OBSTACLE CLEARING
            # =================================================

            if obstacle_latched:

                if (
                    distance_voltage < CLEAR_VOLTAGE
                ):

                    clear_samples += 1

                else:

                    clear_samples = 0


                if (
                    clear_samples
                    >= CLEAR_SAMPLES_REQUIRED
                ):

                    obstacle_latched = False

                    clear_samples = 0

                    target_a = 0.0
                    target_b = 0.0

                    command_a = 0.0
                    command_b = 0.0

                    controller_a.reset()
                    controller_b.reset()

                    rover.stop()

                    mode = "PATH CLEAR"


            # =================================================
            # READ ENCODERS
            # =================================================

            rpm_a, rpm_b = (
                speedometer.sample()
            )


            # =================================================
            # CLOSED LOOP SPEED CONTROL
            # =================================================

            if (
                target_a == 0
                and
                target_b == 0
            ):

                command_a = 0.0
                command_b = 0.0

                rover.stop()


            else:

                command_a = (
                    controller_a.update(
                        target_a,
                        rpm_a,
                        dt
                    )
                )


                command_b = (
                    controller_b.update(
                        target_b,
                        rpm_b,
                        dt
                    )
                )


                rover.set_wheels(
                    command_a,
                    command_b
                )


            # =================================================
            # DISPLAY
            # =================================================

            stdscr.erase()


            stdscr.addstr(
                0,
                0,
                "ROVER - PID + COLLISION CONTROL"
            )


            stdscr.addstr(
                2,
                0,
                f"Mode: {mode}"
            )


            # ------------------------------------------------
            # Wheel A
            # ------------------------------------------------

            stdscr.addstr(
                4,
                0,
                (
                    f"A target: {target_a:+6.1f} rpm"
                    f"   measured: {rpm_a:6.1f}"
                    f"   duty: {command_a:+.3f}"
                )
            )


            # ------------------------------------------------
            # Wheel B
            # ------------------------------------------------

            stdscr.addstr(
                5,
                0,
                (
                    f"B target: {target_b:+6.1f} rpm"
                    f"   measured: {rpm_b:6.1f}"
                    f"   duty: {command_b:+.3f}"
                )
            )


            # ------------------------------------------------
            # Distance sensor
            # ------------------------------------------------

            stdscr.addstr(
                7,
                0,
                (
                    f"IR voltage: "
                    f"{distance_voltage:.3f} V"
                )
            )


            stdscr.addstr(
                8,
                0,
                (
                    f"STOP threshold: "
                    f"{OBSTACLE_VOLTAGE:.2f} V"
                )
            )


            stdscr.addstr(
                9,
                0,
                (
                    "Obstacle latch: "
                    +
                    (
                        "YES"
                        if obstacle_latched
                        else "NO"
                    )
                )
            )


            stdscr.addstr(
                10,
                0,
                (
                    f"Clear samples: "
                    f"{clear_samples}/"
                    f"{CLEAR_SAMPLES_REQUIRED}"
                )
            )


            # ------------------------------------------------
            # Controls
            # ------------------------------------------------

            stdscr.addstr(
                12,
                0,
                f"W = forward ({FORWARD_RPM:.0f} rpm)"
            )


            stdscr.addstr(
                13,
                0,
                (
                    f"S = backward "
                    f"({BACKWARD_A_RPM:.0f}/"
                    f"{BACKWARD_B_RPM:.0f} rpm)"
                )
            )


            stdscr.addstr(
                14,
                0,
                (
                    f"A = left "
                    f"({LEFT_A_RPM:.0f}/"
                    f"{LEFT_B_RPM:.0f})"
                )
            )


            stdscr.addstr(
                15,
                0,
                (
                    f"D = right "
                    f"({RIGHT_A_RPM:.0f}/"
                    f"{RIGHT_B_RPM:.0f})"
                )
            )


            stdscr.addstr(
                16,
                0,
                "Z / C = rotate on spot"
            )


            stdscr.addstr(
                17,
                0,
                "E / SPACE = stop"
            )


            stdscr.addstr(
                18,
                0,
                "R = manual obstacle rearm"
            )


            stdscr.addstr(
                19,
                0,
                "Q = quit"
            )


            stdscr.refresh()


    finally:

        rover.close()

        distance_adc.close()


if __name__ == "__main__":

    curses.wrapper(main)
