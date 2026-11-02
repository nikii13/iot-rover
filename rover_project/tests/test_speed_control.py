"""PID de velocidad: convergencia, anti-windup, objetivo cero, sentido, límites."""
import config
from speed_control import MotorPID, WheelSpeedController
from speedometer import WheelSpeed


def make_pid(**kw):
    p = dict(kp=0.004, ki=0.02, kd=0.0, ff_fwd=0.008, ff_back=0.008, ff_static=0.05, integral_limit=0.4, duty_limit=1.0)
    p.update(kw)
    return MotorPID(**p)


def simulate(pid, target, gain=110.0, tau=0.08, t_end=3.0, dt=0.02, dead=0.08):
    rpm = 0.0
    for _ in range(int(t_end / dt)):
        u = pid.update(target, rpm, dt)
        mag = max(0.0, abs(u) - dead) / (1 - dead)
        rpm += ((gain * mag * (1 if u >= 0 else -1)) - rpm) * dt / tau
    return rpm, u


def test_converges_despite_wrong_feedforward():
    pid = make_pid(ff_fwd=0.003)  # FF muy bajo: el integrador debe compensar
    rpm, _ = simulate(pid, 50.0, t_end=6.0)
    assert abs(rpm - 50.0) < 2.0


def test_reverse():
    rpm, u = simulate(make_pid(), -40.0)
    assert abs(rpm + 40.0) < 2.0 and u < 0


def test_zero_target_resets():
    pid = make_pid()
    simulate(pid, 50.0)
    assert pid.update(0.0, 30.0, 0.02) == 0.0
    assert pid.integral == 0.0


def test_anti_windup_when_motor_cannot_reach():
    pid = make_pid()
    simulate(pid, 500.0, gain=100.0, t_end=5.0)  # inalcanzable
    assert abs(pid.integral) <= 0.4 + 1e-9
    # Al bajar el objetivo, recupera rápido (sin integral acumulada enorme)
    rpm, _ = simulate(pid, 40.0, gain=100.0, t_end=1.5)
    assert abs(rpm - 40.0) < 4.0


def test_output_limited():
    pid = make_pid(duty_limit=0.6)
    u = pid.update(1000.0, 0.0, 0.02)
    assert u <= 0.6


def test_direction_change_resets_integral():
    pid = make_pid()
    simulate(pid, 50.0, t_end=1.0)
    pid.update(-30.0, 40.0, 0.02)
    assert pid.integral <= 0.0


def test_controller_clamps_max_rpm():
    c = WheelSpeedController(config.as_namespace(MAX_RPM=60.0))
    c.update(100.0, -100.0, 0.0, 0.0, 0.02)
    assert c.target_a == 60.0 and c.target_b == -60.0


def test_speedometer_rpm_and_sign_from_command():
    w = WheelSpeed(700, 1.0, 600, sign_from_command=True)
    w.update(0, 0.0)
    assert w.update(70, 0.1, command_sign=-0.5)  # 70 cuentas en 0.1 s = 60 RPM
    assert abs(w.rpm + 60.0) < 1e-6


def test_speedometer_rejects_jumps():
    w = WheelSpeed(700, 1.0, 600, sign_from_command=False)
    w.update(0, 0.0)
    assert not w.update(100000, 0.02)
    assert w.invalid == 1


def test_ff_table_interpolation():
    from speed_control import ff_from_table
    t = [(22.3, 0.3), (31.0, 0.4), (69.6, 1.0)]
    assert abs(ff_from_table(t, 26.65) - 0.35) < 1e-9
    assert abs(ff_from_table(t, -31.0) - 0.4) < 1e-9
    assert ff_from_table(t, 5.0) == 0.3             # por debajo: PWM de arranque
    assert ff_from_table(t, 80.0) > 1.0             # extrapola (luego se limita a DUTY_LIMIT)


def test_pid_with_nonlinear_motor_and_table():
    """Motor muy no lineal (como el B real): con tabla FF converge rápido."""
    import math
    table = [(30.0, 0.1), (106.1, 0.2), (154.9, 0.3), (196.9, 0.5), (234.2, 1.0)]

    def motor(u):  # curva inversa aproximada de la tabla
        from speed_control import ff_from_table
        lo, hi = 0.0, 300.0
        for _ in range(40):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if ff_from_table(table, mid) < abs(u) else (lo, mid)
        return math.copysign(lo, u)

    pid = make_pid(kp=0.001, ki=0.001, ff_table=table)
    rpm = 0.0
    for _ in range(50):  # 1 s
        u = pid.update(40.0, rpm, 0.02)
        rpm += (motor(u) - rpm) * 0.02 / 0.08
    assert abs(rpm - 40.0) < 3.0


def test_acceleration_ramp_and_immediate_stop():
    c = WheelSpeedController(config.as_namespace(MAX_RPM=60.0, MAX_ACCEL_RPM_S=100.0))
    c.update(40.0, 40.0, 0.0, 0.0, 0.1)
    assert c.target_a == 10.0 and c.target_b == 10.0
    for _ in range(5):
        c.update(40.0, 40.0, 0.0, 0.0, 0.1)
    assert c.target_a == 40.0
    assert c.update(0.0, 0.0, 30.0, 30.0, 0.02) == (0.0, 0.0)


def test_ramp_does_not_delay_turning():
    """La rampa afecta al avance, no a la diferencia entre ruedas (el giro)."""
    c = WheelSpeedController(config.as_namespace(MAX_RPM=60.0, MAX_ACCEL_RPM_S=100.0))
    for _ in range(20):
        c.update(30.0, 30.0, 30.0, 30.0, 0.02)
    c.update(40.0, 20.0, 30.0, 30.0, 0.02)
    assert c.target_a - c.target_b == 20.0
