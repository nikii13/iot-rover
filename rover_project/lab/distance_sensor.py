#!/usr/bin/env python3
"""
distance_sensor.py — Sensor de distancia infrarrojo analógico leído con un MCP3008.

Configuración tomada de vuestro rc_pid_lab9_working.py (verificada en el rover):
    gpiozero.MCP3008(channel=0, clock_pin=11, mosi_pin=10, miso_pin=9, select_pin=8)
    tensión = valor * 3.3 V; mediana de las 3 últimas lecturas
    A 20-25 cm medisteis 2.2-2.4 V. Objeto cerca  -> tensión ALTA.
    Obstáculo si tensión >= 2.0 V; despejado si < 1.7 V durante ~1 s.

    s = DistanceSensor(cfg); v = s.read()   # voltios filtrados

Ejecutado directamente: muestra la tensión en vivo y si hay obstáculo (prueba física).
"""

import json
import os
import time
from collections import deque
from statistics import median

import config as _config


class DistanceSensorError(RuntimeError):
    pass


class DistanceSensor:
    def __init__(self, cfg=None, adc=None):
        cfg = cfg or _config
        self.cfg = cfg
        if adc is None:
            try:
                from gpiozero import MCP3008
            except ImportError as e:
                raise DistanceSensorError("Falta gpiozero (sudo apt install python3-gpiozero)") from e
            try:
                adc = MCP3008(channel=cfg.DISTANCE_ADC_CHANNEL, clock_pin=cfg.DISTANCE_CLOCK_PIN,
                              mosi_pin=cfg.DISTANCE_MOSI_PIN, miso_pin=cfg.DISTANCE_MISO_PIN,
                              select_pin=cfg.DISTANCE_SELECT_PIN)
            except Exception as e:
                raise DistanceSensorError(f"No se pudo abrir el MCP3008 del sensor de distancia: {e}") from e
        self.adc = adc
        self.samples = deque(maxlen=max(1, int(cfg.DISTANCE_MEDIAN_SAMPLES)))
        self.failures = 0
        self.voltage = 0.0

    def read(self):
        """Tensión filtrada (mediana) en voltios. Lanza DistanceSensorError si falla repetidamente."""
        try:
            raw = float(self.adc.value)
            self.failures = 0
        except Exception as e:
            self.failures += 1
            if self.failures >= self.cfg.DISTANCE_MAX_CONSECUTIVE_FAILURES:
                raise DistanceSensorError(f"Fallo de lectura del sensor de distancia: {e}") from e
            return self.voltage
        self.samples.append(raw)
        self.voltage = median(self.samples) * self.cfg.DISTANCE_VREF
        return self.voltage

    def close(self):
        try:
            self.adc.close()
        except Exception:
            pass


class DistanceCalibration:
    """Tabla tensión -> distancia medida en el rover (los sensores IR no son lineales).

    points: [(cm, voltios), ...]. La tensión debe BAJAR al aumentar la distancia.
    cm(v) interpola en 1/d (el sensor es aproximadamente V ∝ 1/d), devuelve None si la
    tensión es menor que la del punto más lejano (nada delante dentro del rango)."""

    def __init__(self, points):
        pts = sorted((float(d), float(v)) for d, v in points)
        if len(pts) < 3:
            raise ValueError("La calibración de distancia necesita al menos 3 puntos")
        for (d1, v1), (d2, v2) in zip(pts, pts[1:]):
            if not v2 < v1:
                raise ValueError(
                    f"La tensión no baja con la distancia ({d1:.0f} cm: {v1:.2f} V, {d2:.0f} cm: {v2:.2f} V). "
                    "Repetid la calibración (objeto plano, perpendicular, sin moverlo); por debajo de ~10 cm "
                    "estos sensores dan lecturas engañosas.")
        self.points = pts
        self.min_cm, self.max_cm = pts[0][0], pts[-1][0]

    def cm(self, volts):
        if volts is None:
            return None
        pts = self.points
        if volts >= pts[0][1]:
            return pts[0][0]
        if volts < pts[-1][1]:
            return None
        for (d1, v1), (d2, v2) in zip(pts, pts[1:]):
            if v2 <= volts <= v1:
                f = (v1 - volts) / (v1 - v2)
                inv = 1.0 / d1 + f * (1.0 / d2 - 1.0 / d1)
                return 1.0 / inv
        return None

    def volts(self, cm):
        """Inversa (para comprobaciones): tensión esperada a cierta distancia."""
        pts = self.points
        cm = min(max(cm, pts[0][0]), pts[-1][0])
        for (d1, v1), (d2, v2) in zip(pts, pts[1:]):
            if d1 <= cm <= d2:
                f = (1.0 / cm - 1.0 / d1) / (1.0 / d2 - 1.0 / d1)
                return v1 + f * (v2 - v1)
        return pts[-1][1]

    def save(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump({"points": self.points, "date": time.strftime("%Y-%m-%d %H:%M:%S")}, f, indent=2)

    @classmethod
    def load(cls, path):
        if not os.path.exists(path):
            raise FileNotFoundError(
                f"No existe {path}. Calibrad primero: python3 lab/distance_sensor.py --calibrate")
        with open(path) as f:
            return cls(json.load(f)["points"])


def calibrate(sensor, cfg, distances=None, samples=40):
    distances = distances or cfg.DISTANCE_CALIBRATION_CM
    print("Calibración del sensor de distancia. Usad un objeto plano y grande (p. ej. una caja o el")
    print("robot de delante), perpendicular al sensor. La distancia se mide desde el SENSOR.")
    points = []
    for d in distances:
        input(f"  Objeto a {d} cm del sensor y pulsad Enter...")
        vals = []
        for _ in range(samples):
            vals.append(sensor.read())
            time.sleep(0.02)
        v = median(vals)
        print(f"    {d:>3} cm -> {v:.3f} V (min {min(vals):.3f}, max {max(vals):.3f})")
        points.append((d, v))
    cal = DistanceCalibration(points)
    cal.save(cfg.DISTANCE_CALIBRATION_FILE)
    print(f"Guardada en {cfg.DISTANCE_CALIBRATION_FILE}")
    print(f"Objetivo del convoy {cfg.CONVOY_TARGET_CM:.0f} cm ≈ {cal.volts(cfg.CONVOY_TARGET_CM):.2f} V")
    return cal


def main():
    import argparse

    ap = argparse.ArgumentParser(description="Prueba y calibración del sensor de distancia IR")
    ap.add_argument("--calibrate", action="store_true", help="crear la tabla tensión -> cm")
    args = ap.parse_args()
    c = _config
    s = DistanceSensor()
    try:
        if args.calibrate:
            calibrate(s, c)
            return
        try:
            cal = DistanceCalibration.load(c.DISTANCE_CALIBRATION_FILE)
        except (FileNotFoundError, ValueError):
            cal = None
            print("(sin calibración: solo tensión; calibrad con --calibrate para ver centímetros)")
        print(f"Obstáculo si >= {c.OBSTACLE_VOLTAGE:.2f} V, despejado si < {c.CLEAR_VOLTAGE:.2f} V. Ctrl+C para salir.")
        while True:
            v = s.read()
            state = "OBSTÁCULO" if v >= c.OBSTACLE_VOLTAGE else ("despejado" if v < c.CLEAR_VOLTAGE else "zona intermedia")
            dist = ""
            if cal:
                d = cal.cm(v)
                dist = f"  ≈ {d:5.1f} cm" if d is not None else f"  > {cal.max_cm:.0f} cm"
            print(f"\rtensión = {v:5.3f} V{dist}   {state:<16}", end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
    finally:
        s.close()


if __name__ == "__main__":
    main()
