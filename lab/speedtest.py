#!/usr/bin/env python3
import sys
import time

import encoder_client
from rover import Rover
from speedometer import Speedometer

DUTY = float(sys.argv[1]) if len(sys.argv) > 1 else 0.25
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0

encoder_client.reset()
rover = Rover()
speed = Speedometer()

print("Encoder channels:", encoder_client.diagnostics())
print(f"Open-loop test duty={DUTY:.2f}, duration={DURATION:.1f}s")
print("First run: wheels off the ground.")
print(" time | A rpm | B rpm | A cm | B cm")

try:
    rover.forward(DUTY)
    start = time.monotonic()
    while time.monotonic() - start < DURATION:
        time.sleep(0.5)
        ar, br = speed.sample()
        ac, bc = speed.distance_cm()
        print(f"{time.monotonic()-start:5.1f} | {ar:5.1f} | {br:5.1f} | {ac:5.1f} | {bc:5.1f}")
finally:
    rover.close()

print("Encoder channels:", encoder_client.diagnostics())
