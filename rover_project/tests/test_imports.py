"""Todos los módulos Python se importan sin hardware (sin gpiozero/smbus2 instalados)."""
import importlib

import pytest

MODULES = ["config", "rover", "encoder_client", "encoder_status", "speedometer", "speed_control", "imu",
           "color_sensor", "diag_log", "drive_loop", "line_control", "black_line_follow",
           "color_line_follow", "pidtest", "distance_sensor", "convoy"]


@pytest.mark.parametrize("name", MODULES)
def test_import(name):
    importlib.import_module(name)


def test_config_pins_unique():
    import config as c
    motor = [c.MOTOR_A_PIN_1, c.MOTOR_A_PIN_2, c.MOTOR_B_PIN_1, c.MOTOR_B_PIN_2]
    enc = [c.ENCODER_A_PIN_1, c.ENCODER_A_PIN_2, c.ENCODER_B_PIN_1, c.ENCODER_B_PIN_2]
    dist = [c.DISTANCE_CLOCK_PIN, c.DISTANCE_MOSI_PIN, c.DISTANCE_MISO_PIN, c.DISTANCE_SELECT_PIN]
    assert len(set(motor + enc + dist)) == 12, "un GPIO no puede usarse para dos cosas"
    assert not {2, 3} & set(motor + enc + dist), "GPIO2/3 son SDA/SCL del bus I2C"
