#!/usr/bin/env bash
# Detiene SOLO el encodered de este proyecto (SIGTERM: borra su socket al salir).
# Nunca toca el encoderd del laboratorio (encoderd.service).
set -uo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SOCK="$(cd "$ROOT/lab" && python3 -c 'import config; print(config.ENCODER_SOCKET)')"
if pgrep -x encodered >/dev/null; then
    pkill -TERM -x encodered
    for _ in $(seq 1 30); do pgrep -x encodered >/dev/null || break; sleep 0.1; done
    if pgrep -x encodered >/dev/null; then
        echo "No respondió a SIGTERM; forzando."; pkill -KILL -x encodered; rm -f "$SOCK"
    fi
    echo "encodered detenido."
else
    echo "encodered (de este proyecto) no estaba en ejecución."
    if systemctl is-active --quiet encoderd.service 2>/dev/null; then
        echo "El encoderd del laboratorio sigue activo (no se toca)."
    fi
fi
exit 0
