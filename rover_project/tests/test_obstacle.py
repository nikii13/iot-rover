"""Parada por obstáculo con el sensor de distancia IR (sin hardware)."""

import pytest

import config
from distance_sensor import DistanceSensor, DistanceSensorError
from drive_loop import Command, Context
from line_control import ObstacleGuard
from black_line_follow import BlackLineFollower
from line_control import SurfaceCalibration
from sim_rover import REAL_CURVES, SimDistance, SimWorld, Track, make_runtime, real_rover_config


class FakeADC:
    def __init__(self, values):
        self.values = list(values)

    @property
    def value(self):
        v = self.values.pop(0)
        if isinstance(v, Exception):
            raise v
        return v

    def close(self):
        pass


def test_sensor_voltage_and_median():
    s = DistanceSensor(config.as_namespace(), adc=FakeADC([0.2, 0.9, 0.3]))
    s.read(); s.read()
    assert s.read() == pytest.approx(0.3 * 3.3)   # mediana de 0.2, 0.9, 0.3 -> pico aislado filtrado


def test_sensor_failures():
    s = DistanceSensor(config.as_namespace(DISTANCE_MAX_CONSECUTIVE_FAILURES=2), adc=FakeADC([0.5, OSError(), OSError()]))
    s.read()
    assert s.read() == pytest.approx(0.5 * 3.3)   # un fallo: devuelve la última
    with pytest.raises(DistanceSensorError):
        s.read()


def make_ctx(t, v, dt=0.02):
    return Context(t=t, dt=dt, distance_v=v)


def test_guard_stops_latches_and_resumes_after_clear_time():
    resumed = []
    g = ObstacleGuard(lambda ctx: Command(25, 25, "FOLLOWING"), config.as_namespace(), on_resume=lambda: resumed.append(1))
    assert g(make_ctx(0.0, 0.8)).target_left == 25
    cmd = g(make_ctx(0.02, 2.3))
    assert cmd.state == "OBSTACLE" and cmd.target_left == 0 and not cmd.stop_reason
    t = 0.04
    for _ in range(30):                      # zona intermedia (1.7-2.0 V): sigue bloqueado
        assert g(make_ctx(t, 1.85)).state == "OBSTACLE"; t += 0.02
    for _ in range(45):                      # 0.9 s despejado: todavía no
        assert g(make_ctx(t, 1.0)).state == "OBSTACLE"; t += 0.02
    for _ in range(10):
        cmd = g(make_ctx(t, 1.0)); t += 0.02
    assert cmd.state == "FOLLOWING" and resumed == [1]


def test_guard_does_not_block_spins():
    g = ObstacleGuard(lambda ctx: Command(-30, 30, "SEARCHING"), config.as_namespace())
    assert g(make_ctx(0.0, 2.5)).state == "SEARCHING"


def test_guard_no_resume_ends_program():
    g = ObstacleGuard(lambda ctx: Command(25, 25, "FOLLOWING"), config.as_namespace(OBSTACLE_RESUME=False))
    assert g(make_ctx(0.0, 2.3)).stop_reason


def test_guard_max_wait():
    g = ObstacleGuard(lambda ctx: Command(25, 25, "FOLLOWING"), config.as_namespace(OBSTACLE_MAX_WAIT_S=5.0))
    g(make_ctx(0.0, 2.3))
    assert not g(make_ctx(4.0, 2.3)).stop_reason
    assert "obstáculo" in g(make_ctx(5.1, 2.3)).stop_reason


def _run_with_obstacle(t_off, cfg=None, fail_after=None):
    cfg = cfg or real_rover_config()
    track = Track().straight(80)
    w = SimWorld(track, cfg, dead_ch2=True, motor_curves=REAL_CURVES, load_factor=0.8, tau=0.25, delay_s=0.08)
    dist = SimDistance(w, x=50.0, y=0.0, t_off=t_off, fail_after_s=fail_after)
    rt = make_runtime(w, cfg, imu=False, distance=dist)
    ev = []
    f = BlackLineFollower(SurfaceCalibration(250, 1200), cfg, imu_feedback=False, events=ev.append)
    g = ObstacleGuard(f, cfg, events=ev.append, on_resume=f.machine.resume)
    rt._event = ev.append
    reason = rt.run(g, max_time=60)
    return w, reason, ev, track


def test_rover_stops_before_object_and_resumes_when_removed():
    w, reason, ev, track = _run_with_obstacle(t_off=25.0)
    txt = " | ".join(ev)
    assert "OBSTÁCULO" in txt and "despejado" in txt, txt
    # mientras el objeto estaba, el sensor nunca llegó a tocarlo
    sx_max_before = max(s for t, d, s, c in w.trace if t < 25.0)
    assert sx_max_before < 50.0 - 2.0
    # tras retirarlo, termina la pista
    assert max(s for _, _, s, _ in w.trace) >= track.length - 1, txt
    # la espera parado no se interpretó como línea perdida: solo hay UNA pérdida, la del final de la pista
    losses = [e for e in ev if e.startswith(("FOLLOWING", "CORRECTING")) and "LINE_LOST" in e]
    assert len(losses) == 1 and ev.index(losses[0]) > ev.index(next(e for e in ev if "despejado" in e)), txt


def test_object_that_stays_ends_after_max_wait():
    w, reason, ev, _ = _run_with_obstacle(t_off=1e9, cfg=real_rover_config(OBSTACLE_MAX_WAIT_S=8.0))
    assert "obstáculo durante más de 8" in reason


def test_sensor_failure_is_a_safety_stop():
    w, reason, ev, _ = _run_with_obstacle(t_off=1e9, fail_after=2.0)
    assert "Sensor de distancia" in reason and w.t < 3.0
