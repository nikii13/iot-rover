#!/usr/bin/env python3
"""
Test 2 — Motores y correspondencia motor/encoder/lado. REQUIERE el rover y encodered.

Levantad el rover (ruedas en el aire). Cada motor se mueve 1 s "adelante" con
PWM 0.5. Se comprueba qué contador cambia y se os pregunta qué rueda giró y
hacia dónde. Al final se proponen MOTOR_A_SIDE / MOTOR_*_INVERT.
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lab"))

from encoder_client import EncoderClient, EncoderError  # noqa: E402
from rover import Rover  # noqa: E402


def ask(q, options):
    while True:
        a = input(f"{q} [{'/'.join(options)}]: ").strip().lower()
        if a in options:
            return a


def main():
    input("Ruedas en el aire. Pulsad Enter para empezar...")
    suggestions = []
    with Rover() as rover, EncoderClient(timeout_s=0.2) as enc:
        try:
            for name, duties in (("A", (0.5, 0.0)), ("B", (0.0, 0.5))):
                c0 = enc.get_counts()
                rover.set_motors(*duties)
                time.sleep(1.0)
                rover.stop()
                time.sleep(0.3)
                c1 = enc.get_counts()
                da, db = c1[0] - c0[0], c1[1] - c0[1]
                print(f"Motor {name}: Δcuentas A={da:+d}  B={db:+d}")
                own, other = (da, db) if name == "A" else (db, da)
                if own == 0 and other != 0:
                    print("  ! Cuenta el encoder del OTRO motor: encoders A/B intercambiados en el cableado o en config.")
                elif own == 0:
                    print(f"  ! El encoder {name} no cuenta (ver lab/encoder_status.py).")
                elif own < 0:
                    print("  · Cuentas negativas al ir adelante: canal 2 en fase opuesta (inofensivo con "
                          "ENCODER_SIGN_FROM_COMMAND=True).")
                side = ask(f"¿Qué rueda giró con el motor {name}?", ["izquierda", "derecha", "ninguna"])
                if side == "ninguna":
                    print(f"  ! El motor {name} no se mueve: revisad driver, alimentación y pines {name}.")
                    continue
                direction = ask("¿Hacia dónde (sentido de avance del rover)?", ["adelante", "atras"])
                if name == "A":
                    suggestions.append(f"MOTOR_A_SIDE = \"{'left' if side == 'izquierda' else 'right'}\"")
                suggestions.append(f"MOTOR_{name}_INVERT = {direction == 'atras'}")
        except EncoderError as e:
            print(f"ERROR encoders: {e}")
            return 1
        finally:
            rover.stop()
    print("\nConfiguración propuesta para lab/config_local.py:")
    for s in suggestions:
        print("  " + s)
    return 0


if __name__ == "__main__":
    sys.exit(main())
