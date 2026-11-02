#!/usr/bin/env bash
# Asegura que hay un servicio de encoders respondiendo en ENCODER_SOCKET.
#  - Si ya responde uno (p. ej. el encoderd del laboratorio, servicio systemd
#    encoderd.service), se USA ese y no se arranca nada más.
#  - Solo si no hay ninguno, arranca el encodered de este proyecto (cpp/).
# No requiere sudo: el usuario debe pertenecer al grupo "gpio".
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN="$ROOT/cpp/encodered"
LOGDIR="$ROOT/logs"
mkdir -p "$LOGDIR"

[ -x "$BIN" ] || "$ROOT/scripts/build.sh"

# Parámetros desde la configuración (única fuente de verdad)
eval "$(cd "$ROOT/lab" && python3 - <<'PY'
import config as c
print(f'SOCK="{c.ENCODER_SOCKET}"; LOCK="{c.ENCODER_LOCK}"')
print(f'PINS="{c.ENCODER_A_PIN_1},{c.ENCODER_A_PIN_2},{c.ENCODER_B_PIN_1},{c.ENCODER_B_PIN_2}"')
print(f'MODE="{c.ENCODER_DECODE_MODE}"; BIAS="{c.ENCODER_BIAS}"; DEB="{c.ENCODER_DEBOUNCE_US}"')
PY
)"


if ! id -nG | grep -qw gpio && [ "$(id -u)" -ne 0 ]; then
    echo "AVISO: el usuario $(whoami) no está en el grupo 'gpio'. Ejecutad: sudo usermod -aG gpio $(whoami) y volved a iniciar sesión."
fi

ping_socket() {
    (cd "$ROOT/lab" && python3 -c "
from encoder_client import EncoderClient
c = EncoderClient(timeout_s=0.2, retries=0)
try:
    raise SystemExit(0 if c.ping() or c.get_counts() else 1)
except Exception:
    raise SystemExit(1)
finally:
    c.close()" ) >/dev/null 2>&1
}

if pgrep -x encodered >/dev/null; then
    echo "encodered (de este proyecto) ya está en ejecución (pid $(pgrep -x encodered | tr '\n' ' '))."
    exit 0
fi

# ¿Hay ya un servicio respondiendo (p. ej. el encoderd del laboratorio)? -> usarlo.
if [ -e "$SOCK" ] && ping_socket; then
    if systemctl is-active --quiet encoderd.service 2>/dev/null; then
        echo "Usando el encoderd del laboratorio (encoderd.service activo) en $SOCK. No se arranca nada más."
    else
        echo "Ya hay un servicio de encoders respondiendo en $SOCK (propietario $(stat -c '%U' "$SOCK")). Se usa ese."
    fi
    exit 0
fi

if systemctl is-enabled --quiet encoderd.service 2>/dev/null; then
    echo "ERROR: encoderd.service está instalado pero no responde. Probad: sudo systemctl restart encoderd"
    echo "  (no se arranca encodered: ambos usarían los mismos GPIO)"
    exit 1
fi

# Socket residual sin servicio detrás
if [ -e "$SOCK" ]; then
    if ! rm -f "$SOCK" 2>/dev/null || [ -e "$SOCK" ]; then
        echo "ERROR: $SOCK es un socket residual de $(stat -c '%U' "$SOCK") y no se puede borrar."
        echo "  Borradlo con: sudo rm $SOCK   (y comprobad que no hay otro servicio de encoders)"
        exit 1
    fi
    echo "Socket residual $SOCK eliminado."
fi

nohup "$BIN" --socket "$SOCK" --lock "$LOCK" --pins "$PINS" --mode "$MODE" --bias "$BIAS" \
      --debounce-us "$DEB" >>"$LOGDIR/encodered.log" 2>&1 &
PID=$!
for _ in $(seq 1 50); do
    [ -S "$SOCK" ] && ping_socket && break
    if ! kill -0 "$PID" 2>/dev/null; then
        echo "ERROR: encodered terminó al arrancar. Últimas líneas de $LOGDIR/encodered.log:"
        tail -n 8 "$LOGDIR/encodered.log"
        exit 1
    fi
    sleep 0.1
done
kill -0 "$PID" 2>/dev/null && ping_socket || { echo "ERROR: encodered no responde en $SOCK"; tail -n 8 "$LOGDIR/encodered.log"; exit 1; }
echo "encodered en ejecución (pid $PID), socket $SOCK, pines $PINS, modo $MODE. Log: $LOGDIR/encodered.log"
