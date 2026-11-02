"""Detección y lectura de IMU con un bus I2C simulado (sin hardware)."""
import config
from imu import IMU, detect


class FakeICM20948Bus:
    """Imita un ICM-20948 en 0x69: bancos, WHO_AM_I=0xEA en banco 0, giro Z en 0x37."""

    def __init__(self, gyro_z_raw=0):
        self.bank = 0
        self.writes = []
        self.gz = gyro_z_raw

    def read_byte_data(self, addr, reg):
        if addr != 0x69:
            raise OSError("no device")
        if reg == 0x00 and self.bank == 0:
            return 0xEA
        return 0x40  # otros registros (p. ej. 0x75), como en el rover real

    def write_byte_data(self, addr, reg, val):
        self.writes.append((reg, val))
        if reg == 0x7F:
            self.bank = val >> 4

    def read_i2c_block_data(self, addr, reg, n):
        assert self.bank == 0 and reg == 0x37
        v = self.gz & 0xFFFF
        return [v >> 8, v & 0xFF]

    def close(self):
        pass


def test_detects_icm20948_on_0x69_like_the_real_rover():
    fam, addr, name = detect(FakeICM20948Bus())
    assert name == "ICM-20948" and addr == 0x69


def test_icm20948_rate_bias_and_sign():
    bus = FakeICM20948Bus(gyro_z_raw=655)  # 655 / 65.5 = 10 deg/s
    fam, addr, name = detect(bus)
    cfg = config.as_namespace(IMU_RATE_FILTER_ALPHA=1.0, IMU_YAW_SIGN=-1)
    imu = IMU(fam(bus, addr, "z"), name, cfg)
    assert abs(imu.read_rate() + 10.0) < 1e-6
    imu.bias = 10.0
    assert abs(imu.read_rate()) < 1e-6
    assert (0x7F, 0x20) in bus.writes and bus.bank == 0  # configuró el banco 2 y volvió al 0
