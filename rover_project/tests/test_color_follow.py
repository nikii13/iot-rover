"""Modelo de colores y seguimiento con el mismo bucle (simulador en escala de grises)."""
import pytest

from color_line_follow import ColorLineFollower, ColorModel
from color_sensor import Reading
from line_control import CalibrationError
from sim_rover import SimWorld, Track, make_runtime, sim_config

BG = [1200, 400, 400, 400]
COLORS = {"red": [700, 450, 120, 130], "green": [650, 120, 400, 130], "blue": [600, 110, 140, 350]}


def test_classifies_calibrated_colors():
    m = ColorModel(BG, COLORS)
    for name, v in COLORS.items():
        got, cov, res = m.analyse(Reading(*v))
        assert got == name and cov > 0.95 and res < 0.05


def test_partial_coverage_on_edge():
    m = ColorModel(BG, COLORS)
    half = [(a + b) / 2 for a, b in zip(BG, COLORS["green"])]
    got, cov, _ = m.analyse(Reading(*half))
    assert got == "green" and 0.4 < cov < 0.6


def test_background():
    got, cov, _ = ColorModel(BG, COLORS).analyse(Reading(*BG))
    assert got is None and cov < 0.1


def test_rejects_color_equal_to_background():
    with pytest.raises(CalibrationError):
        ColorModel(BG, {"x": BG})


def test_color_follower_follows_left_curve_and_stop_action():
    # El simulador produce grises: calibramos un "color" igual al negro simulado.
    cfg = sim_config(COLOR_ACTIONS={"dark": {"speed_scale": 0.8, "stop": False}})
    track = Track().straight(30).arc(15, 90).straight(20)
    w = SimWorld(track, cfg)
    rt = make_runtime(w, cfg)
    model = ColorModel([1200, 400, 400, 400], {"dark": [250, 83, 83, 83]})
    ev = []
    reason = rt.run(ColorLineFollower(model, cfg, events=ev.append), max_time=40)
    assert max(s for _, _, s, _ in w.trace) >= track.length - 1, ev
    assert "no encontrada" in reason or "sin éxito" in reason
