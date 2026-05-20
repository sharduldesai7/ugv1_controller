"""
MPU9250 Low-Level Driver
=========================
Handles all I2C register communication with the MPU9250 (MPU6500 + AK8963).

Registers sourced from:
  - MPU-9250 Register Map Rev 1.6
  - AK8963 Register Map

Sensor axes follow ROS REP-103 convention:
  X = forward, Y = left, Z = up  (right-hand coordinate system)

Data cleaning applied:
  - Hardware DLPF (Digital Low Pass Filter) configured at init
  - Gyroscope zero-rate offset calibration (static bias removal)
  - Accelerometer static bias removal
  - Magnetometer hard-iron offset correction
  - Complementary filter for orientation estimate
"""

import smbus2
import time
import struct
import math
from dataclasses import dataclass, field
from typing import Tuple, Optional


# ── MPU6500 register addresses ─────────────────────────────────────────────────
REG_SELF_TEST_X_GYRO  = 0x00
REG_SMPLRT_DIV        = 0x19   # Sample rate divider
REG_CONFIG            = 0x1A   # DLPF config
REG_GYRO_CONFIG       = 0x1B   # Gyro full-scale range
REG_ACCEL_CONFIG      = 0x1C   # Accel full-scale range
REG_ACCEL_CONFIG2     = 0x1D   # Accel DLPF config
REG_INT_PIN_CFG       = 0x37   # Bypass enable for AK8963
REG_INT_ENABLE        = 0x38
REG_ACCEL_XOUT_H      = 0x3B   # Accel data start (6 bytes)
REG_TEMP_OUT_H        = 0x41   # Temperature data (2 bytes)
REG_GYRO_XOUT_H       = 0x43   # Gyro data start (6 bytes)
REG_USER_CTRL         = 0x6A
REG_PWR_MGMT_1        = 0x6B   # Power management
REG_PWR_MGMT_2        = 0x6C
REG_WHO_AM_I          = 0x75   # Should return 0x71 for MPU9250

# ── AK8963 (magnetometer) addresses ───────────────────────────────────────────
AK8963_ADDRESS        = 0x0C
AK8963_WHO_AM_I       = 0x00   # Should return 0x48
AK8963_INFO           = 0x01
AK8963_ST1            = 0x02   # Status 1 (data ready)
AK8963_XOUT_L         = 0x03   # Mag data start (6 bytes, little-endian)
AK8963_ST2            = 0x09   # Status 2 (overflow flag)
AK8963_CNTL1          = 0x0A   # Control 1 (mode, output bits)
AK8963_CNTL2          = 0x0B   # Soft reset
AK8963_ASAX           = 0x10   # Sensitivity adjustment X (factory cal)
AK8963_ASAY           = 0x11
AK8963_ASAZ           = 0x12

# ── Full-scale range options ───────────────────────────────────────────────────
GYRO_RANGE_250DPS  = 0x00  # ±250  °/s  — LSB = 131.0
GYRO_RANGE_500DPS  = 0x08  # ±500  °/s  — LSB = 65.5
GYRO_RANGE_1000DPS = 0x10  # ±1000 °/s  — LSB = 32.8
GYRO_RANGE_2000DPS = 0x18  # ±2000 °/s  — LSB = 16.4

ACCEL_RANGE_2G     = 0x00  # ±2g   — LSB = 16384
ACCEL_RANGE_4G     = 0x08  # ±4g   — LSB = 8192
ACCEL_RANGE_8G     = 0x10  # ±8g   — LSB = 4096
ACCEL_RANGE_16G    = 0x18  # ±16g  — LSB = 2048

# ── DLPF bandwidth options (CONFIG register) ───────────────────────────────────
DLPF_BANDWIDTH_250HZ  = 0x00
DLPF_BANDWIDTH_184HZ  = 0x01
DLPF_BANDWIDTH_92HZ   = 0x02
DLPF_BANDWIDTH_41HZ   = 0x03   # Good default — smooth but not laggy
DLPF_BANDWIDTH_20HZ   = 0x04
DLPF_BANDWIDTH_10HZ   = 0x05
DLPF_BANDWIDTH_5HZ    = 0x06

GRAVITY = 9.80665   # m/s² — standard gravity


@dataclass
class IMUReading:
    """One complete reading from all MPU9250 sensors."""
    # Accelerometer (m/s²)
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0

    # Gyroscope (rad/s)
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0

    # Magnetometer (µT) — AK8963
    mag_x: float = 0.0
    mag_y: float = 0.0
    mag_z: float = 0.0
    mag_valid: bool = False

    # Temperature (°C)
    temperature: float = 0.0

    # Timestamp
    timestamp: float = 0.0


@dataclass
class CalibrationData:
    """Bias offsets computed during calibration."""
    # Gyro zero-rate bias (rad/s) — subtracted from every gyro reading
    gyro_bias_x: float = 0.0
    gyro_bias_y: float = 0.0
    gyro_bias_z: float = 0.0

    # Accel bias (m/s²) — subtracted from every accel reading
    # Z-bias corrects for +1g when sensor is flat
    accel_bias_x: float = 0.0
    accel_bias_y: float = 0.0
    accel_bias_z: float = 0.0

    # Magnetometer hard-iron offsets (µT)
    mag_bias_x: float = 0.0
    mag_bias_y: float = 0.0
    mag_bias_z: float = 0.0

    # AK8963 factory sensitivity adjustment (read from FUSE_ROM)
    mag_scale_x: float = 1.0
    mag_scale_y: float = 1.0
    mag_scale_z: float = 1.0


class MPU9250Driver:
    """
    Low-level driver for the MPU9250 over I2C.

    Usage:
        imu = MPU9250Driver(bus=1, address=0x68)
        imu.initialize()
        imu.calibrate_gyro()     # keep sensor still for 2 seconds
        imu.calibrate_accel()    # keep sensor flat for 2 seconds

        while True:
            reading = imu.read()
            print(reading.accel_x, reading.gyro_z)
    """

    def __init__(
        self,
        bus:           int   = 1,
        address:       int   = 0x68,
        gyro_range:    int   = GYRO_RANGE_500DPS,
        accel_range:   int   = ACCEL_RANGE_4G,
        dlpf_bandwidth: int  = DLPF_BANDWIDTH_41HZ,
    ):
        self.bus_num        = bus
        self.address        = address
        self.gyro_range     = gyro_range
        self.accel_range    = accel_range
        self.dlpf_bandwidth = dlpf_bandwidth

        self._bus  = smbus2.SMBus(bus)
        self._cal  = CalibrationData()

        # Sensitivity scalars — set in initialize() based on range config
        self._gyro_scale  = 1.0   # raw → rad/s
        self._accel_scale = 1.0   # raw → m/s²

    # ── Initialisation ─────────────────────────────────────────────────────────

    def initialize(self) -> bool:
        """
        Wake the MPU9250, configure ranges and DLPF, enable AK8963 bypass.
        Returns True on success, False if WHO_AM_I check fails.
        """
        # WHO_AM_I sanity check — MPU9250 = 0x71, some clones return 0x73
        who = self._read_byte(REG_WHO_AM_I)
        if who not in (0x71, 0x73, 0x70):
            raise RuntimeError(
                f'MPU9250 not found at 0x{self.address:02X} — '
                f'WHO_AM_I returned 0x{who:02X} (expected 0x71)'
            )

        # Wake up — clear sleep bit, select best available clock source
        self._write_byte(REG_PWR_MGMT_1, 0x01)   # PLL with X-gyro ref
        time.sleep(0.1)

        # Enable all accelerometer and gyroscope axes
        self._write_byte(REG_PWR_MGMT_2, 0x00)

        # Sample rate divider — 0 = max rate (1kHz / (1 + 0) = 1kHz)
        self._write_byte(REG_SMPLRT_DIV, 0x00)

        # DLPF — sets bandwidth and noise floor
        self._write_byte(REG_CONFIG, self.dlpf_bandwidth)

        # Gyroscope full-scale range
        self._write_byte(REG_GYRO_CONFIG, self.gyro_range)

        # Accelerometer full-scale range
        self._write_byte(REG_ACCEL_CONFIG, self.accel_range)

        # Accelerometer DLPF — match gyro DLPF setting
        self._write_byte(REG_ACCEL_CONFIG2, self.dlpf_bandwidth)

        # Enable I2C bypass so AK8963 is visible on main bus
        self._write_byte(REG_INT_PIN_CFG, 0x02)
        time.sleep(0.01)

        # Compute sensitivity scalars
        self._gyro_scale  = self._compute_gyro_scale()
        self._accel_scale = self._compute_accel_scale()

        # Initialise AK8963 magnetometer
        self._mag_available = False
        try:
            self._init_magnetometer()
            self._mag_available = True
            print('[IMU] AK8963 magnetometer initialised')
        except OSError as e:
            print(f'[IMU] AK8963 not available (errno {e.errno}) — '
            f'running accel+gyro only')


        return True

    def _init_magnetometer(self):
        """Configure AK8963 for 16-bit continuous measurement at 100Hz."""
        # Reset AK8963
        self._write_ak(AK8963_CNTL2, 0x01)
        time.sleep(0.01)

        # Enter FUSE ROM access mode to read factory sensitivity adjustment
        self._write_ak(AK8963_CNTL1, 0x0F)
        time.sleep(0.01)

        # Read factory sensitivity adjustment values
        asax = self._read_ak(AK8963_ASAX)
        asay = self._read_ak(AK8963_ASAY)
        asaz = self._read_ak(AK8963_ASAZ)

        # Convert to scale factor per AK8963 datasheet formula:
        # Hadj = H × ((ASA - 128) × 0.5 / 128 + 1)
        self._cal.mag_scale_x = (asax - 128) * 0.5 / 128 + 1.0
        self._cal.mag_scale_y = (asay - 128) * 0.5 / 128 + 1.0
        self._cal.mag_scale_z = (asaz - 128) * 0.5 / 128 + 1.0

        # Power down before mode change
        self._write_ak(AK8963_CNTL1, 0x00)
        time.sleep(0.01)

        # Set 16-bit output, continuous measurement mode 2 (100Hz)
        self._write_ak(AK8963_CNTL1, 0x16)
        time.sleep(0.01)

    # ── Calibration ────────────────────────────────────────────────────────────

    def calibrate_gyro(self, num_samples: int = 500, delay: float = 0.002):
        """
        Compute gyroscope zero-rate bias.
        Keep the sensor completely still during this call (~1 second).
        """
        print(f'[IMU] Calibrating gyro — keep sensor still ({num_samples} samples)...')
        sx = sy = sz = 0.0
        for _ in range(num_samples):
            raw = self._read_raw_gyro()
            sx += raw[0]
            sy += raw[1]
            sz += raw[2]
            time.sleep(delay)

        # Bias = mean of raw readings × scale factor
        self._cal.gyro_bias_x = (sx / num_samples) * self._gyro_scale
        self._cal.gyro_bias_y = (sy / num_samples) * self._gyro_scale
        self._cal.gyro_bias_z = (sz / num_samples) * self._gyro_scale
        print(
            f'[IMU] Gyro bias: '
            f'x={self._cal.gyro_bias_x:.4f} '
            f'y={self._cal.gyro_bias_y:.4f} '
            f'z={self._cal.gyro_bias_z:.4f} rad/s'
        )

    def calibrate_accel(self, num_samples: int = 500, delay: float = 0.002):
        """
        Compute accelerometer bias.
        Keep the sensor flat (Z-axis pointing up) during this call.
        The calibration removes X/Y bias and corrects Z to read exactly +g.
        """
        print(f'[IMU] Calibrating accel — keep sensor flat ({num_samples} samples)...')
        sx = sy = sz = 0.0
        for _ in range(num_samples):
            raw = self._read_raw_accel()
            sx += raw[0]
            sy += raw[1]
            sz += raw[2]
            time.sleep(delay)

        ax = (sx / num_samples) * self._accel_scale
        ay = (sy / num_samples) * self._accel_scale
        az = (sz / num_samples) * self._accel_scale

        # X and Y should be 0, Z should be +GRAVITY when flat
        self._cal.accel_bias_x = ax
        self._cal.accel_bias_y = ay
        self._cal.accel_bias_z = az - GRAVITY   # remove gravity from Z

        print(
            f'[IMU] Accel bias: '
            f'x={self._cal.accel_bias_x:.4f} '
            f'y={self._cal.accel_bias_y:.4f} '
            f'z={self._cal.accel_bias_z:.4f} m/s²'
        )

    def calibrate_mag(self, duration: float = 15.0):
        """
        Compute magnetometer hard-iron offsets.
        Slowly rotate the sensor through all orientations during this call.
        Duration in seconds — 15s minimum recommended.
        """
        print(f'[IMU] Calibrating magnetometer — rotate sensor freely for {duration:.0f}s...')
        mx_min = mx_max = None
        my_min = my_max = None
        mz_min = mz_max = None

        t_end = time.time() + duration
        count = 0
        while time.time() < t_end:
            mag = self._read_raw_mag()
            if mag is None:
                time.sleep(0.01)
                continue

            mx, my, mz = mag
            if mx_min is None:
                mx_min = mx_max = mx
                my_min = my_max = my
                mz_min = mz_max = mz
            else:
                mx_min, mx_max = min(mx_min, mx), max(mx_max, mx)
                my_min, my_max = min(my_min, my), max(my_max, my)
                mz_min, mz_max = min(mz_min, mz), max(mz_max, mz)
            count += 1
            time.sleep(0.01)

        if count == 0:
            print('[IMU] Magnetometer calibration failed — no samples received')
            return

        # Hard-iron offset = midpoint of min/max range
        self._cal.mag_bias_x = (mx_max + mx_min) / 2.0
        self._cal.mag_bias_y = (my_max + my_min) / 2.0
        self._cal.mag_bias_z = (mz_max + mz_min) / 2.0

        print(
            f'[IMU] Mag bias: '
            f'x={self._cal.mag_bias_x:.2f} '
            f'y={self._cal.mag_bias_y:.2f} '
            f'z={self._cal.mag_bias_z:.2f} µT'
        )

    def set_calibration(self, cal: CalibrationData):
        """Load previously computed calibration data."""
        self._cal = cal

    def get_calibration(self) -> CalibrationData:
        """Return current calibration data (save to file for persistence)."""
        return self._cal

    # ── Read ───────────────────────────────────────────────────────────────────

    def read(self) -> IMUReading:
        """
        Read all sensors, apply calibration, and return an IMUReading.
        This is the main method to call in your control loop.
        """
        reading = IMUReading(timestamp=time.time())

        # ── Accelerometer ──────────────────────────────────────────────────────
        raw_a = self._read_raw_accel()
        reading.accel_x = raw_a[0] * self._accel_scale - self._cal.accel_bias_x
        reading.accel_y = raw_a[1] * self._accel_scale - self._cal.accel_bias_y
        reading.accel_z = raw_a[2] * self._accel_scale - self._cal.accel_bias_z

        # ── Gyroscope ──────────────────────────────────────────────────────────
        raw_g = self._read_raw_gyro()
        reading.gyro_x = raw_g[0] * self._gyro_scale - self._cal.gyro_bias_x
        reading.gyro_y = raw_g[1] * self._gyro_scale - self._cal.gyro_bias_y
        reading.gyro_z = raw_g[2] * self._gyro_scale - self._cal.gyro_bias_z

        # ── Temperature ────────────────────────────────────────────────────────
        raw_t = self._read_raw_temp()
        # MPU9250 datasheet formula: T(°C) = (raw / 333.87) + 21.0
        reading.temperature = (raw_t / 333.87) + 21.0

        # ── Magnetometer ───────────────────────────────────────────────────────
        raw_m = self._read_raw_mag()
        if raw_m is not None:
            # Apply factory sensitivity adjustment then subtract hard-iron bias
            reading.mag_x = (raw_m[0] * self._cal.mag_scale_x * 0.15) - self._cal.mag_bias_x
            reading.mag_y = (raw_m[1] * self._cal.mag_scale_y * 0.15) - self._cal.mag_bias_y
            reading.mag_z = (raw_m[2] * self._cal.mag_scale_z * 0.15) - self._cal.mag_bias_z
            reading.mag_valid = True
            # 0.15 µT/LSB — sensitivity for 16-bit mode from AK8963 datasheet

        return reading

    # ── Raw register reads ─────────────────────────────────────────────────────

    def _read_raw_accel(self) -> Tuple[int, int, int]:
        """Read 6 bytes of accelerometer data, return signed 16-bit ints."""
        data = self._read_bytes(REG_ACCEL_XOUT_H, 6)
        return self._unpack_xyz(data)

    def _read_raw_gyro(self) -> Tuple[int, int, int]:
        """Read 6 bytes of gyroscope data, return signed 16-bit ints."""
        data = self._read_bytes(REG_GYRO_XOUT_H, 6)
        return self._unpack_xyz(data)

    def _read_raw_temp(self) -> int:
        """Read 2 bytes of temperature data, return signed 16-bit int."""
        data = self._read_bytes(REG_TEMP_OUT_H, 2)
        return struct.unpack('>h', bytes(data))[0]

    def _read_raw_mag(self) -> Optional[Tuple[int, int, int]]:
        """
        Read magnetometer data from AK8963.
        Returns None if data not ready or overflow detected.
        AK8963 data is little-endian (unlike MPU6500 which is big-endian).
        """
        # Check ST1 data-ready bit
        if not self._mag_available:
            return None
        try:
            st1 = self._read_ak(AK8963_ST1)
        except OSError:
            return None
        
        if not (st1 & 0x01):
            return None   # data not ready yet

        # Read 6 bytes (little-endian XYZ)
        data = self._read_ak_bytes(AK8963_XOUT_L, 6)

        # Must read ST2 to latch next measurement and check overflow
        st2 = self._read_ak(AK8963_ST2)
        if st2 & 0x08:
            return None   # magnetic sensor overflow

        x = struct.unpack('<h', bytes([data[0], data[1]]))[0]
        y = struct.unpack('<h', bytes([data[2], data[3]]))[0]
        z = struct.unpack('<h', bytes([data[4], data[5]]))[0]
        return (x, y, z)

    @staticmethod
    def _unpack_xyz(data) -> Tuple[int, int, int]:
        """Unpack 6 bytes of big-endian signed 16-bit XYZ values."""
        x = struct.unpack('>h', bytes([data[0], data[1]]))[0]
        y = struct.unpack('>h', bytes([data[2], data[3]]))[0]
        z = struct.unpack('>h', bytes([data[4], data[5]]))[0]
        return (x, y, z)

    # ── Scale factor computation ───────────────────────────────────────────────

    def _compute_gyro_scale(self) -> float:
        """Return gyro sensitivity in rad/s per LSB."""
        lsb_per_dps = {
            GYRO_RANGE_250DPS:  131.0,
            GYRO_RANGE_500DPS:  65.5,
            GYRO_RANGE_1000DPS: 32.8,
            GYRO_RANGE_2000DPS: 16.4,
        }
        dps_per_lsb = 1.0 / lsb_per_dps[self.gyro_range]
        return math.radians(dps_per_lsb)   # convert to rad/s

    def _compute_accel_scale(self) -> float:
        """Return accel sensitivity in m/s² per LSB."""
        lsb_per_g = {
            ACCEL_RANGE_2G:  16384.0,
            ACCEL_RANGE_4G:  8192.0,
            ACCEL_RANGE_8G:  4096.0,
            ACCEL_RANGE_16G: 2048.0,
        }
        g_per_lsb = 1.0 / lsb_per_g[self.accel_range]
        return g_per_lsb * GRAVITY   # convert to m/s²

    # ── I2C helpers ────────────────────────────────────────────────────────────

    def _write_byte(self, reg: int, value: int):
        self._bus.write_byte_data(self.address, reg, value)

    def _read_byte(self, reg: int) -> int:
        return self._bus.read_byte_data(self.address, reg)

    def _read_bytes(self, reg: int, length: int) -> list:
        return self._bus.read_i2c_block_data(self.address, reg, length)

    def _write_ak(self, reg: int, value: int):
        self._bus.write_byte_data(AK8963_ADDRESS, reg, value)

    def _read_ak(self, reg: int) -> int:
        return self._bus.read_byte_data(AK8963_ADDRESS, reg)

    def _read_ak_bytes(self, reg: int, length: int) -> list:
        return self._bus.read_i2c_block_data(AK8963_ADDRESS, reg, length)

    def close(self):
        self._bus.close()
