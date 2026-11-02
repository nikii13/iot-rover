#!/usr/bin/env python3
"""Envoltorio de prueba física: ejecuta lab/color_sensor.py (mismos argumentos)."""
import os
import runpy
import sys

LAB = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "lab")
sys.path.insert(0, LAB)
sys.argv[0] = os.path.join(LAB, "color_sensor.py")
runpy.run_path(sys.argv[0], run_name="__main__")
