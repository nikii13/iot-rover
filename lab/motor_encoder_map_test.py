#!/usr/bin/env python3

import time
import encoder_client
from rover import Rover

rover = Rover()

try:
    print("\n=== TEST MOTOR A ONLY ===")
    encoder_client.reset()
    time.sleep(0.2)

    rover.set_wheels(0.25, 0.0)
    time.sleep(2)
    rover.stop()

    print("Encoder result:")
    print(encoder_client.diagnostics())

    time.sleep(1)

    print("\n=== TEST MOTOR B ONLY ===")
    encoder_client.reset()
    time.sleep(0.2)

    rover.set_wheels(0.0, 0.25)
    time.sleep(2)
    rover.stop()

    print("Encoder result:")
    print(encoder_client.diagnostics())

finally:
    rover.close()
