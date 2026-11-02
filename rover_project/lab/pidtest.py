#!/usr/bin/env python3
"""
pidtest.py — Test 3 (PID) y medición de feed-forward. ¡Ruedas LEVANTADAS del suelo!

  python3 lab/pidtest.py --rpm 40 --time 6            # ambos motores a 40 RPM
  python3 lab/pidtest.py --rpm 40 --motor a           # solo motor A
  python3 lab/pidtest.py --rpm -30                    # marcha atrás
  python3 lab/pidtest.py --ff-fit                     # mide RPM vs PWM en lazo abierto
                                                      # y propone FF_*_PER_RPM, FF_*_STATIC, MAX_RPM

El test de PID usa el mismo bucle (drive_loop) que los seguidores de línea.
"""

import argparse
import statistics
import sys
import time

import config as _config
from drive_loop import Command, build_runtime


def run_pid(args, cfg):
    rt = build_runtime("pidtest", cfg, need_color=False, need_imu=False, console=not args.quiet)
    ta = args.rpm if args.motor in ("a", "both") else 0.0
    tb = args.rpm if args.motor in ("b", "both") else 0.0
    samples = []

    from rover import side_values

    def step(ctx):
        if ctx.t > args.time / 2:
            samples.append((rt.speedometer.rpm_a, rt.speedometer.rpm_b, rt.rover.duty_a, rt.rover.duty_b))
        left, right = side_values(cfg, ta, tb)
        return Command(left, right, "PIDTEST")

    try:
        reason = rt.run(step, max_time=args.time)
    finally:
        rt.close()
    print(f"\nFin: {reason}")
    if samples:
        for i, (name, tgt) in enumerate((("A", ta), ("B", tb))):
            rpms = [s[i] for s in samples]
            duties = [s[2 + i] for s in samples]
            if tgt:
                print(f"Motor {name}: objetivo {tgt:+.1f} RPM | medido {statistics.mean(rpms):+.1f} ± "
                      f"{statistics.pstdev(rpms):.1f} | PWM medio {statistics.mean(duties):+.2f}")
    return 0


def ff_fit(args, cfg):
    from encoder_client import EncoderClient
    from rover import Rover
    from speedometer import Speedometer

    input("Levantad el rover (ruedas en el aire) y pulsad Enter...")
    duties = [0.05, 0.075, 0.1, 0.125, 0.15, 0.175, 0.2, 0.25, 0.3, 0.35, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
    sp = Speedometer(_config.as_namespace(ENCODER_SIGN_FROM_COMMAND=True))
    results = {"a": [], "b": []}
    with Rover(cfg) as rover, EncoderClient(cfg=cfg) as enc:
        try:
            for motor in ("a", "b"):
                for d in duties:
                    rover.set_motors(d if motor == "a" else 0, d if motor == "b" else 0)
                    t_end = time.monotonic() + 1.2
                    sp.reset(enc.get_counts())
                    t_prev, vals = time.monotonic(), []
                    while time.monotonic() < t_end:
                        time.sleep(0.05)
                        t = time.monotonic()
                        sp.update(enc.get_counts(), t - t_prev, rover.duty_a, rover.duty_b)
                        t_prev = t
                        if t > t_end - 0.6:
                            vals.append(sp.a.rpm_raw if motor == "a" else sp.b.rpm_raw)
                    rpm = statistics.mean(vals)
                    results[motor].append((d, rpm))
                    print(f"motor {motor.upper()} PWM {d:.3f} -> {rpm:7.1f} RPM", flush=True)
                rover.stop()
                time.sleep(0.8)
        finally:
            rover.stop()
    print()
    print("Tablas para lab/config_local.py (feed-forward medido, recomendado):")
    for motor, pts in results.items():
        table, last = [], 0.0
        for d, r in pts:
            if r > 3.0 and r > last + 0.5:  # solo puntos con movimiento y RPM creciente
                table.append((round(r, 1), d))
                last = r
        print(f"FF_{motor.upper()}_TABLE = {table}")
    print()
    for motor, pts in results.items():
        moving = [(d, r) for d, r in pts if r > 3.0]
        if len(moving) < 2:
            print(f"Motor {motor.upper()}: no se mueve o el encoder no cuenta -> revisar con encoder_status.py")
            continue
        n = len(moving)
        mx = sum(d for d, _ in moving) / n
        my = sum(r for _, r in moving) / n
        k = sum((d - mx) * (r - my) for d, r in moving) / sum((d - mx) ** 2 for d, _ in moving)
        d0 = mx - my / k
        M = motor.upper()
        print(f"Motor {M}: RPM ≈ {k:.1f} * (PWM - {d0:.2f})   RPM a PWM 1.0 = {pts[-1][1]:.1f}")
        print(f"  Sugerencia: FF_{M}_FORWARD_PER_RPM = {1/k:.4f}   FF_{M}_STATIC = {max(0.0, d0):.2f}")
    top = min(r for pts in results.values() for _, r in pts[-1:])
    print(f"Sugerencia: MAX_RPM <= {0.85 * top:.0f} (85 % de la RPM máxima del motor más lento)")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rpm", type=float, default=40.0)
    ap.add_argument("--time", type=float, default=6.0)
    ap.add_argument("--motor", choices=["a", "b", "both"], default="both")
    ap.add_argument("--ff-fit", action="store_true")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()
    cfg = _config.as_namespace()
    try:
        return ff_fit(args, cfg) if args.ff_fit else run_pid(args, cfg)
    except Exception as e:
        print(f"ERROR: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
