#!/usr/bin/env python3
"""
color_sensor.py — Driver del sensor de color TCS34725 (I2C, dirección fija 0x29).

Usa smbus2 directamente (sin librerías de Adafruit) para leer los cuatro canales
en una sola transacción y controlar el tiempo de integración, que determina la
frecuencia real de lectura (integración 12 ms ≈ 80 lecturas/s).

    s = ColorSensor(cfg); r = s.read()   ->  Reading(c, r, g, b, fresh, saturated)

Ejecutado directamente (Test 5): muestra lecturas en vivo para comparar negro y fondo.
"""

import time
from dataclasses import dataclass

import config as _config

_CMD = 0x80
_AUTO_INC = 0x20
_ENABLE, _ATIME, _CONTROL, _ID, _STATUS, _CDATAL = 0x00, 0x01, 0x0F, 0x12, 0x13, 0x14
_GAINS = {1: 0x00, 4: 0x01, 16: 0x02, 60: 0x03}
_VALID_IDS = {0x44: "TCS34721/TCS34725", 0x4D: "TCS34723/TCS34727"}


class ColorSensorError(RuntimeError):
    pass


@dataclass
class Reading:
    c: int
    r: int
    g: int
    b: int
    t: float = 0.0          # instante monotónico de la lectura
    fresh: bool = True      # False si el sensor aún no tenía integración nueva
    saturated: bool = False

    def value(self, channel="clear"):
        return float(self.c) if channel == "clear" else float(self.r + self.g + self.b)

    def chroma(self):
        s = float(self.r + self.g + self.b) or 1.0
        return (self.r / s, self.g / s, self.b / s)


class ColorSensor:
    def __init__(self, cfg=None, bus=None):
        cfg = cfg or _config
        self.cfg = cfg
        if bus is None:
            try:
                from smbus2 import SMBus
            except ImportError as e:
                raise ColorSensorError("Falta smbus2 (pip install smbus2)") from e
            bus = SMBus(cfg.COLOR_I2C_BUS)
        self.bus = bus
        self.addr = cfg.COLOR_I2C_ADDRESS
        try:
            ident = self._r8(_ID)
        except OSError as e:
            raise ColorSensorError(
                f"No responde el sensor de color en I2C bus {cfg.COLOR_I2C_BUS} dirección 0x{self.addr:02X}: {e}. "
                "¿I2C activado (raspi-config) y cableado correcto?") from e
        if ident not in _VALID_IDS:
            raise ColorSensorError(f"ID inesperado 0x{ident:02X}: el dispositivo en 0x{self.addr:02X} no es un TCS3472x")
        self.model = _VALID_IDS[ident]
        steps = max(1, min(256, int(round(cfg.COLOR_INTEGRATION_MS / 2.4))))
        self.integration_ms = steps * 2.4
        self.max_count = min(65535, steps * 1024)
        if cfg.COLOR_GAIN not in _GAINS:
            raise ColorSensorError(f"COLOR_GAIN debe ser uno de {sorted(_GAINS)}")
        self._w8(_ATIME, 256 - steps)
        self._w8(_CONTROL, _GAINS[cfg.COLOR_GAIN])
        self._w8(_ENABLE, 0x01)          # PON
        time.sleep(0.003)
        self._w8(_ENABLE, 0x03)          # PON + AEN
        time.sleep(self.integration_ms / 1000.0 + 0.003)
        self.failures = 0
        self.last = None

    def _r8(self, reg):
        return self.bus.read_byte_data(self.addr, _CMD | reg)

    def _w8(self, reg, val):
        self.bus.write_byte_data(self.addr, _CMD | reg, val)

    def read(self):
        """Lee C,R,G,B. Si no hay integración nueva devuelve la anterior con fresh=False."""
        try:
            status = self._r8(_STATUS)
            if not (status & 0x01) and self.last is not None:
                self.failures = 0
                prev = self.last
                return Reading(prev.c, prev.r, prev.g, prev.b, prev.t, False, prev.saturated)
            d = self.bus.read_i2c_block_data(self.addr, _CMD | _AUTO_INC | _CDATAL, 8)
        except OSError as e:
            self.failures += 1
            if self.failures >= self.cfg.COLOR_MAX_CONSECUTIVE_FAILURES or self.last is None:
                raise ColorSensorError(f"Fallo de lectura del sensor de color: {e}") from e
            return Reading(self.last.c, self.last.r, self.last.g, self.last.b, self.last.t, False, self.last.saturated)
        self.failures = 0
        c, r, g, b = (d[i] | (d[i + 1] << 8) for i in range(0, 8, 2))
        self.last = Reading(c, r, g, b, time.monotonic(), True, c >= self.max_count)
        return self.last

    def close(self):
        try:
            self._w8(_ENABLE, 0x00)
        except Exception:
            pass
        try:
            self.bus.close()
        except Exception:
            pass


def main():
    import argparse

    ap = argparse.ArgumentParser(description="Test 5: lecturas del sensor de color")
    ap.add_argument("--hz", type=float, default=10)
    args = ap.parse_args()
    s = ColorSensor()
    print(f"Sensor {s.model}: integración {s.integration_ms:.1f} ms, ganancia {_config.COLOR_GAIN}x, "
          f"máximo {s.max_count} cuentas")
    print("Colocad el sensor sobre la línea negra, el fondo y el borde. Ctrl+C para salir.")
    n, t0 = 0, time.monotonic()
    try:
        while True:
            r = s.read()
            n += r.fresh
            ch = r.chroma()
            sat = "  SATURADO: bajar ganancia/integración" if r.saturated else ""
            print(f"\rC={r.c:6d} R={r.r:6d} G={r.g:6d} B={r.b:6d}  rgb%=({ch[0]:.2f},{ch[1]:.2f},{ch[2]:.2f})"
                  f"  lecturas/s={n / max(1e-3, time.monotonic() - t0):5.1f}{sat}   ", end="", flush=True)
            time.sleep(1.0 / args.hz)
    except KeyboardInterrupt:
        print()
    finally:
        s.close()


if __name__ == "__main__":
    main()
