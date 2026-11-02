"""Calibración, histéresis, signos del seguidor de borde y mezcla con IMU."""
import pytest

import config
from color_sensor import Reading
from line_control import (CalibrationError, DriveMixer, EdgeFollower, SurfaceCalibration,
                          SurfaceClassifier)


def R(c):
    return Reading(c, 0, 0, 0)


def test_coverage_and_validation():
    cal = SurfaceCalibration(200, 1000).validate()
    assert cal.coverage(R(1000)) == 0.0
    assert cal.coverage(R(200)) == 1.0
    assert abs(cal.coverage(R(600)) - 0.5) < 1e-9
    assert cal.coverage(R(5000)) == 0.0 and cal.coverage(R(0)) == 1.0


def test_validation_rejects_bad_calibration():
    with pytest.raises(CalibrationError):
        SurfaceCalibration(1000, 200).validate()          # intercambiado
    with pytest.raises(CalibrationError):
        SurfaceCalibration(950, 1000).validate()          # sin contraste


def test_calibration_roundtrip(tmp_path):
    p = str(tmp_path / "s.json")
    SurfaceCalibration(210, 990).save(p)
    c = SurfaceCalibration.load(p)
    assert (c.line, c.background) == (210, 990)


def test_sweep_calibration_uses_percentiles():
    vals = [1000] * 80 + [200] * 20 + [5, 3000]  # dos valores anómalos
    c = SurfaceCalibration.from_sweep(vals)
    assert c.line == 200 and c.background == 1000


def test_hysteresis():
    cl = SurfaceClassifier()
    assert cl.update(0.9) == "LINE"
    assert cl.update(0.75) == "LINE"            # dentro de la histéresis
    assert cl.update(0.5) == "EDGE"
    assert cl.update(0.1) == "BACKGROUND"
    assert cl.update(0.2) == "BACKGROUND"       # no sale hasta WHITE_EXIT
    assert cl.update(0.3) == "EDGE"


@pytest.mark.parametrize("edge", ["left", "right"])
def test_edge_follower_signs(edge):
    """Borde izquierdo: oscuro -> izquierda (+). Borde derecho: al revés. Ambos sentidos de curva."""
    f = EdgeFollower(config.as_namespace(EDGE_SIDE=edge))
    _, w_dark, _ = f.update(0.9, 0.02)
    f.reset()
    _, w_light, _ = f.update(0.1, 0.02)
    s = 1 if edge == "left" else -1
    assert s * w_dark > 0 and s * w_light < 0
    assert f.default_search_direction() == -s


def test_edge_follower_slows_down_in_corrections():
    f = EdgeFollower()
    v0, _, _ = f.update(0.5, 0.02)
    f.reset()
    v1, _, _ = f.update(0.95, 0.02)
    assert v1 < v0 and v1 >= config.MIN_FOLLOW_RPM


def test_mixer_imu_tracks_requested_turn_not_initial_heading():
    """La IMU no se opone a una curva: si el rover gira menos de lo pedido, refuerza el giro."""
    m = DriveMixer(config.as_namespace(MAX_RPM=90.0), imu_feedback=True)
    l0, r0 = m.mix(40, 60.0, omega_meas=60.0)    # gira lo pedido: sin corrección
    assert abs(m.last_correction) < 1e-9
    l1, r1 = m.mix(40, 60.0, omega_meas=20.0)    # gira menos: más diferencia hacia la izquierda
    assert (r1 - l1) > (r0 - l0) > 0
    l2, r2 = m.mix(40, 0.0, omega_meas=-10.0)    # recta con deriva a la derecha: corrige a la izquierda
    assert r2 > l2


def test_mixer_saturation_preserves_turn():
    m = DriveMixer(config.as_namespace(MAX_RPM=60.0), imu_feedback=False)
    l, r = m.mix(60, 40.0)
    assert max(abs(l), abs(r)) <= 60.0 + 1e-9
    assert r - l == pytest.approx(m.k_diff * 40.0)


def test_mixer_no_reverse_option():
    m = DriveMixer(config.as_namespace(ALLOW_WHEEL_REVERSE=False), imu_feedback=False)
    l, r = m.mix(10, 150.0)
    assert l >= 0 and r > 0


def test_repeated_losses_stop_instead_of_looping():
    """Perder y encontrar la línea en bucle sin avanzar termina en STOPPED."""
    from drive_loop import Context
    from line_control import LineFollowStateMachine
    m = LineFollowStateMachine(config.as_namespace(MAX_LOSSES=3, LOSS_WINDOW_S=10.0, DARKNESS_FILTER_ALPHA=1.0))
    ctx, t, reason = Context(dt=0.02), 0.0, None
    for cycle in range(2000):
        t += 0.02
        ctx.t = t
        cov = 0.5 if (cycle // 50) % 2 == 0 else 0.0   # 1 s sobre el borde, 1 s en blanco
        cmd = m.step(cov, ctx)
        if cmd.stop_reason:
            reason = cmd.stop_reason
            break
    assert reason and "pérdidas de línea" in reason
