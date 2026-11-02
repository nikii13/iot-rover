"""
Seguimiento de línea negra en el simulador (sin hardware), usando el bucle real
(drive_loop) y el seguidor real (black_line_follow). Cubre el fallo observado:
primera curva hacia la IZQUIERDA, y su simétrica.

Estas pruebas detectan errores de lógica; no sustituyen a las pruebas físicas.
"""
import pytest

from black_line_follow import BlackLineFollower
from line_control import SurfaceCalibration
from sim_rover import SimWorld, Track, make_runtime, sim_config

TRACKS = {
    "curva_izquierda_primero": lambda: Track().straight(30).arc(15, 90).straight(30),
    "curva_derecha_primero": lambda: Track().straight(30).arc(15, -90).straight(30),
    "S_izq_der": lambda: Track().straight(20).arc(15, 90).arc(15, -180).straight(20),
    "izquierda_120_R8": lambda: Track().straight(20).arc(8, 120).straight(20),
    "derecha_120_R8": lambda: Track().straight(20).arc(8, -120).straight(20),
    "cerrada_izquierda_R5": lambda: Track().straight(20).arc(5, 90).straight(20),
    "cerrada_derecha_R5": lambda: Track().straight(20).arc(5, -90).straight(20),
}


def run(track, edge="left", imu=True, max_time=60, cfg_over=None, **world_kw):
    cfg = sim_config(EDGE_SIDE=edge, **(cfg_over or {}))
    w = SimWorld(track, cfg, edge=edge, **world_kw)
    rt = make_runtime(w, cfg, imu=imu)
    events = []
    follower = BlackLineFollower(SurfaceCalibration(250, 1200), cfg, imu_feedback=imu, events=events.append)
    reason = rt.run(follower, max_time=max_time)
    return w, reason, events


def tracking(w, track):
    """Desviación máxima del sensor respecto al borde y progreso, hasta llegar al final."""
    pts = []
    for t, d, s, c in w.trace:
        if s >= track.length - 1:
            break
        pts.append(d)
    progress = max(s for _, _, s, _ in w.trace)
    return max(pts) - track.width / 2, progress


@pytest.mark.parametrize("edge", ["left", "right"])
@pytest.mark.parametrize("name", list(TRACKS))
def test_follows_curves_both_directions(name, edge):
    track = TRACKS[name]()
    w, reason, events = run(track, edge)
    dev, progress = tracking(w, track)
    assert progress >= track.length - 1, f"no llegó al final ({progress:.1f}/{track.length:.1f}); eventos={events}"
    assert dev < 1.5, f"se alejó {dev:.2f} cm de la línea"
    # Al acabarse la línea: busca de forma acotada y se detiene (no sigue recto ni gira sin fin)
    assert "LINE_LOST" in " ".join(events)
    assert "no encontrada" in reason or "búsqueda sin éxito" in reason, reason


def test_left_curve_without_imu():
    track = TRACKS["curva_izquierda_primero"]()
    w, reason, events = run(track, "left", imu=False)
    assert tracking(w, track)[1] >= track.length - 1


def test_left_curve_with_dead_encoder_channel():
    """Canal 2 sin señal: cuentas siempre positivas; el sentido sale de la orden aplicada."""
    track = TRACKS["cerrada_izquierda_R5"]()
    w, reason, events = run(track, "left", dead_ch2=True)
    assert tracking(w, track)[1] >= track.length - 1


def test_starts_off_line_and_finds_it():
    """Arranca 1.5 cm a la izquierda de la línea, en fondo claro: debe buscar y encontrarla."""
    track = Track().straight(40)
    w, reason, events = run(track, "left", start_lateral=2.5, max_time=20)
    txt = " ".join(events)
    assert "SEARCHING" in txt and "línea encontrada" in txt, events
    assert max(s for _, _, s, _ in w.trace) > 30


def test_search_is_bounded_in_angle_and_time():
    """Línea muy corta: tras perderla, la búsqueda no supera los barridos configurados."""
    import math
    track = Track().straight(8)
    cfg_over = dict(SEARCH_TIMEOUT_S=6.0)
    w, reason, events = run(track, "left", max_time=30, cfg_over=cfg_over)
    assert w.t < 20, "debe parar mucho antes del tiempo máximo"
    assert "STOPPED" in " ".join(events)
    assert abs(math.degrees(w.th)) < 120, "la búsqueda no debe dar vueltas completas"
