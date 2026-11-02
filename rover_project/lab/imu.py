#!/usr/bin/env python3
"""
imu.py — Velocidad angular de guiñada (yaw rate) y ángulo integrado desde la IMU.

El modelo no se supone: se detecta leyendo el registro WHO_AM_I de las familias
soportadas en el bus I2C. Si vuestra IMU no es de estas familias, el programa
lo indica y se puede ejecutar con IMU_MODEL = "none" (se usa la odometría de
los encoders como respaldo) o añadir aquí un driver con la misma interfaz.

Familias soportadas (registros públicos de los fabricantes):
  - InvenSense MPU-6050/6500/9250/9255/6886, ICM-20602/20689  (0x68/0x69, WHO_AM_I 0x75)
  - ST LSM6DS3/DS33/DSL/DSO/DSOX                               (0x6A/0x6B, WHO_AM_I 0x0F)
  - TDK InvenSense ICM-20948 (registros por bancos)            (0x68/0x69, WHO_AM_I 0x00 = 0xEA)
    -> la del rover NikiC: 0x69, WHO_AM_I 0xEA (medido 29-10)

Interfaz usada por el resto del proyecto:
    imu = open_imu(cfg)        # None si IMU_MODEL="none" o no se detecta y no es obligatoria
    imu.calibrate()            # rover QUIETO: estima el bias del giróscopo
    rate = imu.read_rate()     # deg/s, positivo = giro a la izquierda (tras IMU_YAW_SIGN)
    imu.yaw_deg                # ángulo integrado desde reset_yaw()

Tratamiento del drift: bias medido en reposo + banda muerta pequeña. El ángulo
integrado solo se usa en ventanas cortas (búsqueda de línea, calibración), nunca
como rumbo absoluto que haya que mantener indefinidamente.
"""

import time

import config as _config


class IMUError(RuntimeError):
    pass


def _s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v & 0x8000 else v


class _MPUFamily:
    ADDRESSES = (0x68, 0x69)
    WHO_AM_I_REG = 0x75
    IDS = {0x68: "MPU-6050", 0x70: "MPU-6500", 0x71: "MPU-9250", 0x73: "MPU-9255",
           0x19: "MPU-6886", 0x12: "ICM-20602", 0x98: "ICM-20689"}
    AXIS_REG = {"x": 0x43, "y": 0x45, "z": 0x47}
    SENS = 65.5  # LSB por deg/s en ±500 dps

    def __init__(self, bus, addr, axis):
        self.bus, self.addr = bus, addr
        self.reg = self.AXIS_REG[axis]
        bus.write_byte_data(addr, 0x6B, 0x01)  # PWR_MGMT_1: despertar, reloj PLL giróscopo X
        time.sleep(0.05)
        bus.write_byte_data(addr, 0x1A, 0x03)  # CONFIG: DLPF ~44 Hz
        bus.write_byte_data(addr, 0x19, 0x04)  # SMPLRT_DIV: 200 Hz
        bus.write_byte_data(addr, 0x1B, 0x08)  # GYRO_CONFIG: ±500 dps

    def raw_rate(self):
        hi, lo = self.bus.read_i2c_block_data(self.addr, self.reg, 2)
        return _s16(hi, lo) / self.SENS


class _LSM6Family:
    ADDRESSES = (0x6A, 0x6B)
    WHO_AM_I_REG = 0x0F
    IDS = {0x69: "LSM6DS3/DS33", 0x6A: "LSM6DSL", 0x6C: "LSM6DSO/DSOX"}
    AXIS_REG = {"x": 0x22, "y": 0x24, "z": 0x26}
    SENS = 1000.0 / 17.5  # LSB por deg/s en ±500 dps

    def __init__(self, bus, addr, axis):
        self.bus, self.addr = bus, addr
        self.reg = self.AXIS_REG[axis]
        bus.write_byte_data(addr, 0x12, 0x44)  # CTRL3_C: BDU + auto-incremento
        bus.write_byte_data(addr, 0x11, 0x54)  # CTRL2_G: 208 Hz, ±500 dps
        time.sleep(0.05)

    def raw_rate(self):
        lo, hi = self.bus.read_i2c_block_data(self.addr, self.reg, 2)
        return _s16(hi, lo) / self.SENS


class _ICM20948:
    """ICM-20948: registros en 4 bancos (REG_BANK_SEL 0x7F). Solo se usa el giróscopo."""
    ADDRESSES = (0x69, 0x68)
    WHO_AM_I_REG = 0x00
    IDS = {0xEA: "ICM-20948"}
    AXIS_REG = {"x": 0x33, "y": 0x35, "z": 0x37}   # banco 0, big-endian
    SENS = 65.5  # LSB por deg/s en ±500 dps

    def __init__(self, bus, addr, axis):
        self.bus, self.addr = bus, addr
        self.reg = self.AXIS_REG[axis]
        self._bank(0)
        bus.write_byte_data(addr, 0x06, 0x80)  # PWR_MGMT_1: reset
        time.sleep(0.05)
        self._bank(0)
        bus.write_byte_data(addr, 0x06, 0x01)  # PWR_MGMT_1: despertar, reloj automático
        bus.write_byte_data(addr, 0x07, 0x00)  # PWR_MGMT_2: acelerómetro y giróscopo activos
        time.sleep(0.02)
        self._bank(2)
        bus.write_byte_data(addr, 0x00, 0x04)  # GYRO_SMPLRT_DIV: 1125/(1+4) = 225 Hz
        bus.write_byte_data(addr, 0x01, (3 << 3) | (1 << 1) | 1)  # GYRO_CONFIG_1: DLPF 3 (~51 Hz), ±500 dps
        self._bank(0)
        time.sleep(0.02)

    def _bank(self, n):
        self.bus.write_byte_data(self.addr, 0x7F, n << 4)

    def raw_rate(self):
        hi, lo = self.bus.read_i2c_block_data(self.addr, self.reg, 2)
        return _s16(hi, lo) / self.SENS


_FAMILIES = {"mpu6050": _MPUFamily, "lsm6ds": _LSM6Family, "icm20948": _ICM20948}


def detect(bus, model="auto", address=None):
    """Devuelve (clase, dirección, nombre) del primer dispositivo reconocido."""
    families = _FAMILIES.items() if model == "auto" else [(model, _FAMILIES[model])]
    for _, fam in families:
        for addr in ([address] if address else fam.ADDRESSES):
            try:
                who = bus.read_byte_data(addr, fam.WHO_AM_I_REG)
            except OSError:
                continue
            if who in fam.IDS:
                return fam, addr, fam.IDS[who]
    return None


class IMU:
    def __init__(self, driver, name, cfg=None):
        cfg = cfg or _config
        self.cfg = cfg
        self.driver = driver
        self.name = name
        self.sign = 1.0 if cfg.IMU_YAW_SIGN >= 0 else -1.0
        self.bias = 0.0
        self.deadband = cfg.IMU_GYRO_DEADBAND_DPS
        self.alpha = cfg.IMU_RATE_FILTER_ALPHA
        self.rate = 0.0
        self.yaw_deg = 0.0
        self.failures = 0
        self.calibrated = False

    def calibrate(self, samples=None, period_s=0.005):
        """Rover QUIETO. Estima el bias y la desviación del giróscopo."""
        n = samples or self.cfg.IMU_CALIBRATION_SAMPLES
        vals = []
        for _ in range(n):
            vals.append(self.driver.raw_rate())
            time.sleep(period_s)
        mean = sum(vals) / len(vals)
        std = (sum((v - mean) ** 2 for v in vals) / len(vals)) ** 0.5
        if std > 3.0:
            raise IMUError(f"El rover parece moverse durante la calibración (desv. {std:.2f} deg/s). Repetir quieto.")
        self.bias = mean
        self.calibrated = True
        self.rate = 0.0
        return mean, std

    def read_rate(self):
        """Velocidad angular filtrada en deg/s (+ = izquierda). Lanza IMUError si falla repetidamente."""
        try:
            r = (self.driver.raw_rate() - self.bias) * self.sign
            self.failures = 0
        except OSError as e:
            self.failures += 1
            if self.failures >= self.cfg.IMU_MAX_CONSECUTIVE_FAILURES:
                raise IMUError(f"IMU sin respuesta ({e})") from e
            return self.rate
        if abs(r) < self.deadband:
            r = 0.0
        self.rate = self.alpha * r + (1 - self.alpha) * self.rate
        return self.rate

    def integrate(self, dt):
        self.yaw_deg += self.rate * dt
        return self.yaw_deg

    def reset_yaw(self):
        self.yaw_deg = 0.0

    def close(self):
        try:
            self.driver.bus.close()
        except Exception:
            pass


def open_imu(cfg=None):
    """Abre la IMU según la configuración. Devuelve None si no hay/está desactivada."""
    cfg = cfg or _config
    if cfg.IMU_MODEL == "none":
        return None
    try:
        from smbus2 import SMBus
    except ImportError as e:
        if cfg.IMU_REQUIRED:
            raise IMUError("Falta smbus2 (pip install smbus2)") from e
        return None
    bus = SMBus(cfg.IMU_I2C_BUS)
    found = detect(bus, cfg.IMU_MODEL, cfg.IMU_I2C_ADDRESS)
    if not found:
        bus.close()
        msg = (f"No se reconoce ninguna IMU en el bus I2C {cfg.IMU_I2C_BUS} (modelo={cfg.IMU_MODEL}). "
               "Ejecutad 'i2cdetect -y 1' y revisad README §IMU.")
        if cfg.IMU_REQUIRED:
            raise IMUError(msg)
        print("[imu] AVISO: " + msg + " Se continuará sin IMU.")
        return None
    fam, addr, name = found
    print(f"[imu] Detectada {name} en 0x{addr:02X}")
    return IMU(fam(bus, addr, cfg.IMU_YAW_AXIS), name, cfg)


def main():
    """Test 4: calibración y lectura de la IMU."""
    cfg = _config.as_namespace(IMU_REQUIRED=True)
    imu = open_imu(cfg)
    print("Calibrando: no mováis el rover...")
    bias, std = imu.calibrate()
    print(f"bias={bias:.3f} deg/s  ruido={std:.3f} deg/s")
    print("Girad el rover A MANO hacia la IZQUIERDA: el ángulo debe AUMENTAR. Ctrl+C para salir.")
    t_prev = time.monotonic()
    try:
        while True:
            time.sleep(0.02)
            t = time.monotonic()
            imu.read_rate()
            imu.integrate(t - t_prev)
            t_prev = t
            print(f"\rrate={imu.rate:8.2f} deg/s   yaw={imu.yaw_deg:8.1f} deg   ", end="", flush=True)
    except KeyboardInterrupt:
        print()
        if imu.yaw_deg < -20:
            print("El ángulo disminuyó al girar a la izquierda -> poned IMU_YAW_SIGN = -1")
    finally:
        imu.close()


if __name__ == "__main__":
    main()
