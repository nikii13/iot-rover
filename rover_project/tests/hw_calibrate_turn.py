#!/usr/bin/env python3
"""
Calibración de giro (rover EN EL SUELO, con espacio). REQUIERE encodered e IMU.

Gira sobre sí mismo a la izquierda durante 3 s con ±RPM en las ruedas y mide
con la IMU la velocidad angular real. Calcula RPM_PER_DEGPS (incluye el
deslizamiento real de las ruedas) y comprueba IMU_YAW_SIGN / MOTOR_A_SIDE.
"""
import argparse
import os
import statistics
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lab"))

import config  # noqa: E402
from drive_loop import Command, build_runtime  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rpm", type=float, default=30.0)
    ap.add_argument("--time", type=float, default=3.0)
    args = ap.parse_args()
    cfg = config.as_namespace(IMU_REQUIRED=True)
    rt = build_runtime("calibrate_turn", cfg, need_color=False, console=False)
    samples = []
    try:
        print("Calibrando IMU: no mováis el rover...")
        rt.calibrate_imu()

        def step(ctx):
            if ctx.t > 1.0:
                samples.append((ctx.omega_meas, ctx.rpm_right - ctx.rpm_left))
            return Command(-args.rpm, args.rpm, "TURN_TEST")

        print(f"Girando a la IZQUIERDA {args.time:.0f} s...")
        print("Fin:", rt.run(step, max_time=args.time))
    finally:
        rt.close()
    if not samples:
        return 1
    w = statistics.mean(s[0] for s in samples)
    d = statistics.mean(s[1] for s in samples)
    turned_left = input("¿El rover giró a la IZQUIERDA (antihorario visto desde arriba)? [s/n]: ").strip().lower() == "s"
    print(f"omega IMU media = {w:+.1f} deg/s, diferencia RPM medida (der-izq) = {d:+.1f}")
    if not turned_left:
        print("! Giró a la derecha: MOTOR_A_SIDE está invertido. Cambiadlo y repetid.")
        return 1
    if w < 0:
        print("! La IMU da negativo girando a la izquierda: poned IMU_YAW_SIGN = -1 y repetid.")
        return 1
    if abs(w) < 5:
        print("! Giro demasiado lento para medir; aumentad --rpm.")
        return 1
    print(f"Sugerencia: RPM_PER_DEGPS = {d / w:.3f}   (teórico con TRACK_WIDTH_CM: {config.rpm_per_degps():.3f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
