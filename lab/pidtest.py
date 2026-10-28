#!/usr/bin/env python3

import csv
import sys
import time

import config
import encoder_client

from rover import Rover
from speedometer import Speedometer
from speed_control import WheelController


# ============================================================
# ARGUMENTS
# ============================================================

TARGET_A = (
    float(sys.argv[1])
    if len(sys.argv) > 1
    else 50.0
)

TARGET_B = (
    float(sys.argv[2])
    if len(sys.argv) > 2
    else 50.0
)

DURATION = (
    float(sys.argv[3])
    if len(sys.argv) > 3
    else 10.0
)


DT = config.CONTROL_INTERVAL


# ============================================================
# SETUP
# ============================================================

encoder_client.reset()

rover = Rover()

speed = Speedometer()

controller_a = WheelController("A")
controller_b = WheelController("B")


rows = []


print(
    f"Targets: "
    f"A={TARGET_A:+.1f} rpm "
    f"B={TARGET_B:+.1f} rpm"
)

print(
    "Positive = physical forward"
)

print(
    "Negative = physical backward"
)

print()

print(
    " time | A rpm A duty | B rpm B duty"
)


start = time.monotonic()
last = start


try:

    while True:

        now = time.monotonic()

        elapsed = (
            now - start
        )


        if elapsed >= DURATION:

            break


        dt = (
            now - last
        )


        if dt < DT:

            time.sleep(
                DT - dt
            )

            now = time.monotonic()

            dt = (
                now - last
            )


        last = now


        # ----------------------------------------------------
        # Encoder measurement
        # ----------------------------------------------------

        rpm_a, rpm_b = (
            speed.sample()
        )


        # ----------------------------------------------------
        # Controllers
        # ----------------------------------------------------

        command_a = (
            controller_a.update(
                TARGET_A,
                rpm_a,
                dt
            )
        )


        command_b = (
            controller_b.update(
                TARGET_B,
                rpm_b,
                dt
            )
        )


        # ----------------------------------------------------
        # Motors
        # ----------------------------------------------------

        rover.set_wheels(
            command_a,
            command_b
        )


        print(
            f"{elapsed:5.2f} | "
            f"{rpm_a:5.1f} {command_a:6.3f} | "
            f"{rpm_b:5.1f} {command_b:6.3f}"
        )


        rows.append(
            {
                "time": elapsed,

                "target_a": TARGET_A,
                "rpm_a": rpm_a,
                "duty_a": command_a,

                "target_b": TARGET_B,
                "rpm_b": rpm_b,
                "duty_b": command_b,
            }
        )


finally:

    rover.close()


# ============================================================
# CSV
# ============================================================

filename = (
    f"pid_A{int(TARGET_A)}_"
    f"B{int(TARGET_B)}.csv"
)


with open(
    filename,
    "w",
    newline=""
) as f:

    writer = csv.DictWriter(
        f,
        fieldnames=[
            "time",
            "target_a",
            "rpm_a",
            "duty_a",
            "target_b",
            "rpm_b",
            "duty_b",
        ]
    )

    writer.writeheader()

    writer.writerows(rows)


print()

print(
    f"Wrote {filename}"
)

print(
    "Encoder channels:",
    encoder_client.diagnostics()
)
