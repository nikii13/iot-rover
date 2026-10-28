#!/usr/bin/env python3
import time
import encoder_client

print("Ctrl+C to stop")
try:
    while True:
        print(encoder_client.diagnostics())
        time.sleep(0.5)
except KeyboardInterrupt:
    pass
