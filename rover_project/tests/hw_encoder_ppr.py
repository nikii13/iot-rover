#!/usr/bin/env python3
"""
Medición de cuentas por vuelta de RUEDA (verifica PULSES_PER_REV_A/B). REQUIERE encodered.

Marcad la rueda con cinta. Giradla A MANO exactamente N vueltas (por defecto 10)
y pulsad Enter. Se muestran las cuentas por vuelta con el modo de decodificación
actual del servicio (STATUS). Repetid con la otra rueda.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lab"))

import config  # noqa: E402
from encoder_client import EncoderClient  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--turns", type=float, default=10.0)
    args = ap.parse_args()
    with EncoderClient(timeout_s=0.3) as enc:
        for name, idx, cfg_ppr in (("A", 0, config.PULSES_PER_REV_A), ("B", 1, config.PULSES_PER_REV_B)):
            input(f"\nRueda del motor {name}: pulsad Enter, girad {args.turns:g} vueltas y volved a pulsar Enter.")
            s0 = enc.status()
            input("  ...girando. Enter al terminar: ")
            s1 = enc.status()
            key = "a" if name == "A" else "b"
            counts = abs((s1["a"], s1["b"])[idx] - (s0["a"], s0["b"])[idx])
            ch1 = s1[key + "1"]["rising"] - s0[key + "1"]["rising"]
            ch2 = s1[key + "2"]["rising"] - s0[key + "2"]["rising"]
            ppr = counts / args.turns
            print(f"  modo={s1.get('mode')} cuentas={counts} -> {ppr:.1f} cuentas/vuelta (config: {cfg_ppr})")
            print(f"  flancos de subida canal1={ch1} ({ch1/args.turns:.1f}/vuelta)  canal2={ch2} ({ch2/args.turns:.1f}/vuelta)")
            if cfg_ppr and abs(ppr - cfg_ppr) / cfg_ppr > 0.05:
                print(f"  ! Difiere más de un 5 %: poned PULSES_PER_REV_{name} = {round(ppr)} en config_local.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
