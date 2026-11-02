#!/usr/bin/env python3
"""
convoy.py — Lab 11: elegir si este rover es LÍDER o SEGUIDOR del convoy.

  Seguidor (sigue la línea y mantiene 20-40 cm, objetivo CONVOY_TARGET_CM = 30, con el de delante):
      python3 lab/convoy.py --role follower

  Líder que sigue la línea (con parada por obstáculo). Mejor despacio, para que el convoy le siga:
      python3 lab/convoy.py --role leader --leader-mode line --base-rpm 22

  Líder conducido a mano (Bluefruit / teclado):
      python3 lab/convoy.py --role leader --leader-mode manual
      -> indica qué programa usar (vuestro programa RC del Lab 9).

Cualquier otra opción se pasa tal cual a black_line_follow.py (p. ej. --max-time 180, --edge right,
--no-imu, --perfil rapido, --base-rpm 22).

Requisitos del seguidor: calibración de la línea (black_line_follow.py --calibrate) y del sensor de
distancia (distance_sensor.py --calibrate).
"""

import argparse
import os
import sys

import black_line_follow

RC_PROGRAM = os.path.expanduser("~/cpsy-encoder-final/cpsy-encoder-final/lab/rc_pid_lab9_working.py")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--role", choices=["leader", "follower"], required=True)
    ap.add_argument("--leader-mode", choices=["line", "manual"], default="line")
    args, rest = ap.parse_known_args(argv)

    if args.role == "follower":
        print("ROL: SEGUIDOR — sigue la línea y regula la velocidad con la distancia al de delante")
        return black_line_follow.main(["--convoy"] + rest)

    if args.leader_mode == "line":
        print("ROL: LÍDER — sigue la línea (con parada por obstáculo)")
        if not any(a.startswith("--base-rpm") or a.startswith("--perfil") for a in rest):
            print("  Consejo: el convoy es más preciso si el líder va despacio: añadid --base-rpm 22")
        return black_line_follow.main(rest)

    print("ROL: LÍDER MANUAL")
    print("  Este proyecto no incluye control por Bluefruit. Usad vuestro programa RC del Lab 9, que ya")
    print("  conduce con PID y para ante obstáculos:")
    if os.path.exists(RC_PROGRAM):
        print(f"    cd {os.path.dirname(RC_PROGRAM)} && python3 {os.path.basename(RC_PROGRAM)}")
    else:
        print("    python3 rc_pid_lab9_working.py   (en su carpeta del laboratorio)")
    print("  Conducid despacio (las curvas suaves y sin frenazos bruscos ayudan a los seguidores).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
