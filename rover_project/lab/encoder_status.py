#!/usr/bin/env python3
"""
encoder_status.py — Test 1 (encoders). Muestra cuentas y flancos por canal.

Uso:  python3 lab/encoder_status.py          (girad cada rueda a mano)
      python3 lab/encoder_status.py --once

Diagnóstico automático:
  - Un canal sin flancos mientras el otro cuenta => canal muerto / mal cableado.
  - Con canal 2 muerto, use ENCODER_DECODE_MODE "x1" o "pulse" (nunca "x4").
"""

import argparse
import sys
import time

from encoder_client import EncoderClient, EncoderError


def diagnose(st):
    if "a1" not in st:
        return ["El servicio no da datos por canal (solo totales A,B)."]
    msgs = []
    for motor, (c1, c2) in (("A", ("a1", "a2")), ("B", ("b1", "b2"))):
        e1 = sum(st[c1].values())
        e2 = sum(st[c2].values())
        if e1 == 0 and e2 == 0:
            msgs.append(f"Motor {motor}: ningún canal ha cambiado (¿rueda parada, encoder sin alimentación o pines erróneos?)")
        elif e2 == 0:
            msgs.append(f"Motor {motor}: canal 2 SIN flancos (la velocidad sigue siendo válida con el canal 1; el sentido sale de la orden al motor).")
        elif e1 == 0:
            msgs.append(f"Motor {motor}: canal 1 SIN flancos (con el encoderd del laboratorio el canal 2 cubre la velocidad; revisad el cableado).")
        elif max(e1, e2) > 3 * min(e1, e2):
            msgs.append(f"Motor {motor}: canales muy desiguales ({e1} vs {e2}) -> posible ruido o mala conexión.")
    if st.get("lost", 0):
        msgs.append(f"Eventos GPIO perdidos: {st['lost']} (pruebe --debounce-us o revise ruido).")
    return msgs


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--once", action="store_true")
    ap.add_argument("--hz", type=float, default=5.0)
    args = ap.parse_args()

    try:
        client = EncoderClient(timeout_s=0.2)
    except OSError as e:
        print(f"No se pudo crear el socket cliente: {e}")
        return 1
    try:
        while True:
            try:
                st = client.status()
            except EncoderError as e:
                print(f"ERROR: {e}")
                return 1
            if "a1" not in st:
                print(f"A={st['a']:>8} B={st['b']:>8} (servicio sin diagnóstico por canal)", flush=True)
                if args.once:
                    return 0
                time.sleep(1.0 / args.hz)
                continue
            line = (f"A={st['a']:>8} B={st['b']:>8} [{st.get('service')}, {st.get('mode')}] | "
                    f"A1 {st['a1']['rising']}/{st['a1']['falling']}  A2 {st['a2']['rising']}/{st['a2']['falling']}  "
                    f"B1 {st['b1']['rising']}/{st['b1']['falling']}  B2 {st['b2']['rising']}/{st['b2']['falling']}  "
                    f"perdidos={st.get('lost', 0)}")
            print(line, flush=True)
            if args.once:
                for m in diagnose(st):
                    print("  ! " + m)
                return 0
            time.sleep(1.0 / args.hz)
    except KeyboardInterrupt:
        print("\nResumen:")
        try:
            for m in diagnose(client.status()) or ["Todos los canales muestran actividad."]:
                print("  - " + m)
        except EncoderError:
            pass
        return 0
    finally:
        client.close()


if __name__ == "__main__":
    sys.exit(main())
