#!/usr/bin/env bash
# Compila el servicio de encoders y ejecuta sus pruebas unitarias.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
make -C "$ROOT/cpp" all test
echo "OK: $ROOT/cpp/encodered"
