"""Convoy (Lab 11): calibración de distancia, control de separación y simulación con líder."""
import pytest

import config
from black_line_follow import BlackLineFollower
from distance_sensor import DistanceCalibration
from drive_loop import Command, Context
from line_control import ConvoyController, LineFollowStateMachine, SurfaceCalibration
from sim_rover import (REAL_CURVES, SimConvoyDistance, SimLeader, SimWorld, Track, convoy_test_calibration,
                       make_runtime, real_rover_config)

CAL = [(15, 3.45), (20, 2.59), (25, 2.07), (30, 1.73), (40, 1.29), (60, 0.86)]


def test_calibration_interpolates_and_detects_out_of_range():
    c = DistanceCalibration(CAL)
    assert c.cm(1.73) == pytest.approx(30.0)
    assert 25 < c.cm(1.9) < 30
    assert c.cm(5.0) == 15            # más cerca que el primer punto: el mínimo
    assert c.cm(0.5) is None          # nada delante
    assert c.volts(c.cm(2.3)) == pytest.approx(2.3, abs=1e-6)


def test_calibration_rejects_non_monotonic(tmp_path):
    with pytest.raises(ValueError):
        DistanceCalibration([(15, 2.0), (20, 2.5), (30, 1.5)])
    p = str(tmp_path / "d.json")
    DistanceCalibration(CAL).save(p)
    assert DistanceCalibration.load(p).points[0] == (15.0, 3.45)


class FakeStep:
    def __init__(self):
        self.calls = 0

    def __call__(self, ctx):
        self.calls += 1
        return Command(25, 25, "FOLLOWING", {})


def make(cfg=None):
    cfg = cfg or config.as_namespace()
    m = LineFollowStateMachine(cfg)
    m.state = "FOLLOWING"
    st = FakeStep()
    return ConvoyController(st, m, DistanceCalibration(CAL), cfg), st, m


def ctx(t, volts, rpm=0.0, dt=0.02):
    return Context(t=t, dt=dt, distance_v=volts, rpm_left=rpm, rpm_right=rpm)


def test_waits_close_moves_far_emergency_stop():
    cc, st, m = make()
    cal = DistanceCalibration(CAL)
    assert cc(ctx(0.0, cal.volts(30))).state == cc.WAIT            # a la distancia objetivo: quieto
    assert cc(ctx(0.02, cal.volts(50))).state == "FOLLOWING"       # lejos: avanza
    for i in range(30):                                             # (la distancia al alejarse se filtra)
        cc(ctx(0.04 + 0.02 * i, cal.volts(50)))
    assert m.speed_scale > 1.0                                      # y más rápido que la base para alcanzarlo
    assert cc(ctx(1.0, cal.volts(17))).state == cc.TOO_CLOSE       # demasiado cerca: parada inmediata


def test_start_stop_hysteresis():
    cc, st, m = make()
    cal = DistanceCalibration(CAL)
    cc(ctx(0.0, cal.volts(50)))
    assert cc.mode == cc.MOVING
    # justo por encima del objetivo: v pequeña pero por encima de STOP_RPM -> sigue en marcha
    t = 0.02
    cmd = None
    for _ in range(5):
        cmd = cc(ctx(t, cal.volts(30.2))); t += 0.02
    assert cmd.state in (cc.WAIT, "FOLLOWING")


def test_leader_out_of_beam_is_assumed_stopped():
    cc, st, m = make()
    cal = DistanceCalibration(CAL)
    cc(ctx(0.0, cal.volts(35)))
    # deja de verse; avanzamos con las ruedas a 26 RPM: la distancia estimada baja
    t, stopped = 0.02, False
    for _ in range(400):
        cmd = cc(ctx(t, 0.3, rpm=26.0 if cc.mode == cc.MOVING else 0.0)); t += 0.02
        if cmd.state == cc.WAIT:
            stopped = True
            break
    assert stopped and cc.source == "estimada"
    assert cc.memory_cm >= config.CONVOY_STOP_CM


def _convoy(track, profile, gap0=45.0, tau=0.25, delay=0.08, max_time=60):
    # mismos ajustes que aplica black_line_follow.py --convoy
    cfg = real_rover_config(SEARCH_FIRST_SWEEP_DEG=config.CONVOY_SEARCH_FIRST_SWEEP_DEG,
                            SEARCH_SECOND_SWEEP_DEG=config.CONVOY_SEARCH_SECOND_SWEEP_DEG,
                            SEARCH_TIMEOUT_S=config.CONVOY_SEARCH_TIMEOUT_S)
    w = SimWorld(track, cfg, dead_ch2=True, motor_curves=REAL_CURVES, load_factor=0.8, tau=tau, delay_s=delay,
                 breakaway={"a": 0.42, "b": 0.25}, footprint_r=0.75)
    leader = SimLeader(track, 2.0 + gap0, profile)
    dist = SimConvoyDistance(w, leader)
    rt = make_runtime(w, cfg, imu=False, distance=dist)
    f = BlackLineFollower(SurfaceCalibration(250, 1200), cfg, imu_feedback=False)
    rt.run(ConvoyController(f, f.machine, convoy_test_calibration(), cfg), max_time=max_time)
    return w, dist


def test_straight_keeps_20_to_40_cm_and_stops_with_leader():
    w, dist = _convoy(Track().straight(400), [(15, 4.0), (22, 0.0), (40, 4.0), (60, 0.0)], gap0=50, max_time=50)
    cruise = [d for t, d in dist.gaps if 8 < t < 15 or 32 < t < 40]
    stopped = [d for t, d in dist.gaps if 20 < t < 22]
    assert 20 <= min(cruise) and max(cruise) <= 40
    assert 20 <= stopped[-1] <= 40
    assert min(d for _, d in dist.gaps) > 18


@pytest.mark.parametrize("name,track", [
    ("curvas", Track().straight(60).arc(20, 90).straight(40).arc(15, -90).straight(150)),
    ("S", Track().straight(40).arc(15, 90).arc(15, -180).straight(150)),
    ("derecha", Track().straight(50).arc(18, -120).straight(150)),
])
def test_curves_follow_without_collision(name, track):
    """El líder entra en curvas (sale del haz del IR), se para y arranca: sin choques, sin perder
    la línea, y al final parado a 20-40 cm del líder."""
    w, dist = _convoy(track, [(8, 5.0), (14, 0.0), (24, 6.0), (28, 0.0), (55, 4.5)], max_time=70)
    assert min(d for _, d in dist.gaps) > 12, "demasiado cerca del líder"
    assert dist.gaps[-1][1] >= 20, "nunca debe acabar encima del líder"
    assert max(s for _, _, s, _ in w.trace) > 150, "no debe perder la línea en las curvas"


def test_role_launcher_imports():
    import convoy
    assert callable(convoy.main)
