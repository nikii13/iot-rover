"""Paradas seguras: encoders caídos, motor sin respuesta, negro continuo, objetivo cero."""
from black_line_follow import BlackLineFollower
from drive_loop import Command
from line_control import SurfaceCalibration
from sim_rover import SimWorld, Track, make_runtime, sim_config


def test_encoder_service_failure_stops_motors():
    cfg = sim_config()
    w = SimWorld(Track().straight(200), cfg)
    rt = make_runtime(w, cfg, encoder_fail_after=1.0)
    f = BlackLineFollower(SurfaceCalibration(250, 1200), cfg)
    reason = rt.run(f, max_time=10)
    assert "Encoders" in reason and w.t < 1.5
    assert rt.rover.duty_a == 0 and rt.rover.duty_b == 0


def test_dead_motor_triggers_stall_stop():
    cfg = sim_config()
    w = SimWorld(Track().straight(200), cfg)
    w.motor_dead["a"] = True
    rt = make_runtime(w, cfg)
    reason = rt.run(lambda ctx: Command(50, 50, "TEST"), max_time=10)
    assert "Motor A" in reason and w.t < 5
    assert rt.rover.duty_a == 0


def test_continuous_black_stops():
    cfg = sim_config(BLACK_SATURATED_MAX_S=1.0)
    track = Track(width_cm=40).straight(200)  # zona negra muy ancha
    w = SimWorld(track, cfg, start_lateral=0.0)
    rt = make_runtime(w, cfg)
    reason = rt.run(BlackLineFollower(SurfaceCalibration(250, 1200), cfg), max_time=20)
    assert "saturada" in reason


def test_exception_in_step_stops_motors():
    cfg = sim_config()
    w = SimWorld(Track().straight(200), cfg)
    rt = make_runtime(w, cfg)

    def bad(ctx):
        if ctx.t > 0.5:
            raise ValueError("fallo de prueba")
        return Command(40, 40, "TEST")

    try:
        rt.run(bad, max_time=5)
    except ValueError:
        pass
    assert rt.rover.duty_a == 0 and rt.rover.duty_b == 0
    assert "fallo de prueba" in rt.stop_reason
