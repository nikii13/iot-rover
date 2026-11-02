#!/usr/bin/env bash
# Diagnóstico rápido del sistema (no mueve motores).
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
echo "== Usuario y grupos";      id
echo "== Modelo";                tr -d '\0' </proc/device-tree/model 2>/dev/null; echo
echo "== gpiochips";             ls -l /dev/gpiochip* 2>/dev/null
echo "== I2C (TCS34725 = 0x29; IMU MPU 0x68/0x69, LSM6DS 0x6a/0x6b)"
if command -v i2cdetect >/dev/null; then i2cdetect -y 1; else echo "instalad i2c-tools"; fi
echo "== Python";                python3 -c "from importlib.metadata import version as v; import gpiozero, smbus2; print('gpiozero', v('gpiozero'), '| smbus2', v('smbus2'))" 2>&1 | tail -1
echo "== encoderd del laboratorio"; systemctl is-active encoderd.service 2>/dev/null || echo "no activo"
echo "== encodered (este proyecto)"; pgrep -a -x encodered || echo "no en ejecución"
SOCK="$(cd "$ROOT/lab" && python3 -c 'import config; print(config.ENCODER_SOCKET)')"
ls -l "$SOCK" 2>/dev/null || echo "sin socket $SOCK"
[ -S "$SOCK" ] && (cd "$ROOT/lab" && python3 encoder_status.py --once)
