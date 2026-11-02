"""
Simulación con las curvas de motor MEDIDAS en el rover NikiC (29-10) y con retardo
y lentitud reales de los motores. Con los parámetros anteriores (LINE_KP_DPS=260,
marcha atrás permitida, línea perdida a los 0.12 s) este modelo reproduce el zigzag
en el sitio observado en la pista; con los actuales debe seguir la línea.
"""
import pytest

from black_line_follow import BlackLineFollower
from line_control import SurfaceCalibration
from sim_rover import REAL_CURVES, SimWorld, Track, make_runtime, real_rover_config

TRACKS = {
    "recta": lambda: Track().straight(80),
    "curva_izquierda": lambda: Track().straight(30).arc(15, 90).straight(30),
    "curva_derecha": lambda: Track().straight(30).arc(15, -90).straight(30),
    "S": lambda: Track().straight(20).arc(12, 90).arc(12, -180).straight(20),
    "cerrada_izquierda_R7": lambda: Track().straight(20).arc(7, 90).straight(20),
    "cerrada_derecha_R7": lambda: Track().straight(20).arc(7, -90).straight(20),
    "derecha_180_R10": lambda: Track().straight(20).arc(10, -180).straight(20),
    "izquierda_180_R10": lambda: Track().straight(20).arc(10, 180).straight(20),
}


def run(track, cfg, imu=False, tau=0.25, delay=0.08, **world):
    w = SimWorld(track, cfg, dead_ch2=True, motor_curves=REAL_CURVES, load_factor=0.8, tau=tau, delay_s=delay,
                 **world)
    rt = make_runtime(w, cfg, imu=imu)
    ev = []
    f = BlackLineFollower(SurfaceCalibration(250, 1200), cfg, imu_feedback=imu, events=ev.append)
    rt.on_imu_disabled = lambda: setattr(f.machine.mixer, "imu_feedback", False)
    reason = rt.run(f, max_time=40)
    return w, reason, ev, rt


@pytest.mark.parametrize("name", list(TRACKS))
def test_real_motors_follow_track(name):
    track = TRACKS[name]()
    w, reason, ev, _ = run(track, real_rover_config())
    assert max(s for _, _, s, _ in w.trace) >= track.length - 1, (reason, ev[:6])


@pytest.mark.parametrize("name", ["curva_derecha", "curva_izquierda", "cerrada_derecha_R7", "S"])
def test_wheel_that_does_not_break_away_under_load(name):
    """Prueba del 29-10: en la curva a la derecha la rueda izquierda (B) se quedaba parada
    con PWM 0.14 al pivotar. Con rozamiento estático alto, el impulso de arranque debe
    sacarla y el rover debe completar la curva."""
    track = TRACKS[name]()
    w, reason, ev, _ = run(track, real_rover_config(), breakaway={"a": 0.42, "b": 0.25})
    assert max(s for _, _, s, _ in w.trace) >= track.length - 1, (reason, ev[:6])


def test_without_kick_the_right_curve_stalls():
    """Documenta la causa del fallo observado."""
    track = TRACKS["curva_derecha"]()
    w, reason, ev, _ = run(track, real_rover_config(STICTION_KICK_RATE=0.0), breakaway={"a": 0.42, "b": 0.25})
    assert max(s for _, _, s, _ in w.trace) < track.length - 1


def test_old_tuning_reproduces_zigzag():
    """Documenta la causa: con la configuración anterior el rover no avanza."""
    old = real_rover_config(LINE_KP_DPS=260.0, LINE_KD_DPS=6.0, OMEGA_MAX_DPS=160.0, BASE_RPM=30.0,
                            ALLOW_WHEEL_REVERSE=True, LOST_CONFIRM_S=0.12, LOST_TURN_DEG=999.0)
    track = TRACKS["recta"]()
    w, reason, ev, _ = run(track, old)
    assert max(s for _, _, s, _ in w.trace) < track.length - 1


def test_inverted_imu_sign_is_detected_and_disabled():
    track = TRACKS["curva_izquierda"]()
    cfg = real_rover_config()
    w, reason, ev, rt = run(track, cfg, imu=True)
    assert max(s for _, _, s, _ in w.trace) >= track.length - 1   # con signo correcto, sigue

    # IMU montada al revés (signo invertido): debe detectarse y desactivarse
    import sim_rover

    class InvertedIMU(sim_rover.SimIMU):
        def read_rate(self):
            return -super().read_rate()

    w2 = SimWorld(track, cfg, dead_ch2=True, motor_curves=REAL_CURVES, load_factor=0.8, tau=0.25, delay_s=0.08)
    rt2 = make_runtime(w2, cfg, imu=True)
    rt2.imu = InvertedIMU(w2)
    ev2 = []
    rt2.logger = type("L", (), {"event": lambda self, t: ev2.append(t), "log": lambda self, r: None})()
    f2 = BlackLineFollower(SurfaceCalibration(250, 1200), cfg, imu_feedback=True, events=ev2.append)
    rt2.on_imu_disabled = lambda: setattr(f2.machine.mixer, "imu_feedback", False)
    rt2.run(f2, max_time=40)
    assert any("IMU_YAW_SIGN" in e for e in ev2), ev2[:5]
    assert rt2.imu is None


def test_integral_keeps_sensor_on_edge_in_long_right_curve():
    """Prueba del 29-10: con el borde izquierdo, las curvas a la derecha costaban.
    Con el término integral el sensor vuelve al borde durante la curva (no se queda junto al blanco)."""
    track = TRACKS["derecha_180_R10"]()

    def mean_cov_in_curve(cfg):
        w, reason, ev, _ = run(track, cfg)
        covs = [c for _, _, s, c in w.trace if 25 < s < 20 + 31.4 - 5]
        return sum(covs) / max(1, len(covs)), max(s for _, _, s, _ in w.trace)

    cov_p, prog_p = mean_cov_in_curve(real_rover_config(LINE_KI_DPS=0.0))
    cov_pi, prog_pi = mean_cov_in_curve(real_rover_config())
    assert prog_pi >= track.length - 1
    assert abs(cov_pi - 0.5) < abs(cov_p - 0.5)


def _straight_oscillation_mm(cfg):
    """Oscilación lateral (RMS, mm) del sensor respecto al borde en una recta, con un sensor
    de zona de transición realista (~7-8 mm de radio, estimado de los logs del 29-10)."""
    import math
    track = Track().straight(90)
    w = SimWorld(track, cfg, dead_ch2=True, motor_curves=REAL_CURVES, load_factor=0.8, tau=0.25, delay_s=0.08,
                 breakaway={"a": 0.42, "b": 0.25}, footprint_r=0.75)
    rt = make_runtime(w, cfg, imu=False)
    rt.run(BlackLineFollower(SurfaceCalibration(250, 1200), cfg, imu_feedback=False), max_time=40)
    lat = [abs(d - track.width / 2) for _, d, s, _ in w.trace if 15 < s < 80]
    return 10 * math.sqrt(sum(x * x for x in lat) / len(lat))


def test_02_11_tuning_reduces_oscillation():
    """KD=6 + suavizado de velocidad (02-11) oscila menos que la configuración anterior."""
    old = real_rover_config(LINE_KD_DPS=0.0, SPEED_TAU_UP_S=0.0, SPEED_TAU_DOWN_S=0.0)
    new = real_rover_config()
    assert _straight_oscillation_mm(new) < _straight_oscillation_mm(old)


def test_speed_smoothing_rises_slowly_and_drops_fast():
    from line_control import EdgeFollower
    f = EdgeFollower(real_rover_config(SPEED_TAU_UP_S=0.8, SPEED_TAU_DOWN_S=0.1))
    v0, _, _ = f.update(0.5, 0.02)             # en el borde: velocidad alta
    v_drop, _, _ = f.update(0.95, 0.02)        # corrección fuerte: baja casi de inmediato
    v_up, _, _ = f.update(0.5, 0.02)           # vuelve al borde: sube despacio
    assert v_drop < v0 and v_up < v0 - 0.5 * (v0 - v_drop)
