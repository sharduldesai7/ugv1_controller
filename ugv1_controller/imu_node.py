#!/usr/bin/env python3
"""
ROS2 IMU Node — MPU9250
========================
Reads from the MPU9250 driver and publishes standard ROS2 messages.

Publishes:
  /imu/data_raw      (sensor_msgs/Imu)            — accel + gyro, no orientation
  /imu/mag           (sensor_msgs/MagneticField)   — magnetometer
  /imu/temperature   (sensor_msgs/Temperature)     — chip temperature

Parameters:
  i2c_bus        (int)    1        I2C bus number
  i2c_address    (int)    0x68     MPU9250 I2C address
  publish_rate   (int)    100      Hz — how fast to publish
  frame_id       (str)    imu_link TF frame
  gyro_range     (str)    500      dps — 250|500|1000|2000
  accel_range    (str)    4        g   — 2|4|8|16
  dlpf_bw        (int)    41       Hz  — 5|10|20|41|92|184|250
  calibrate_gyro (bool)   true     run gyro calibration on startup
  calibrate_accel(bool)   false    run accel calibration on startup
  cal_gyro_bias_x  (float) 0.0    pre-loaded gyro bias (rad/s)
  cal_gyro_bias_y  (float) 0.0
  cal_gyro_bias_z  (float) 0.0
  cal_accel_bias_x (float) 0.0    pre-loaded accel bias (m/s²)
  cal_accel_bias_y (float) 0.0
  cal_accel_bias_z (float) 0.0

Switching calibration mode live:
  ros2 service call /imu/calibrate_gyro  std_srvs/srv/Trigger
  ros2 service call /imu/calibrate_accel std_srvs/srv/Trigger
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from geometry_msgs.msg import Vector3

import math

from .mpu9250_driver import (
    MPU9250Driver,
    CalibrationData,
    GYRO_RANGE_250DPS,
    GYRO_RANGE_500DPS,
    GYRO_RANGE_1000DPS,
    GYRO_RANGE_2000DPS,
    ACCEL_RANGE_2G,
    ACCEL_RANGE_4G,
    ACCEL_RANGE_8G,
    ACCEL_RANGE_16G,
    DLPF_BANDWIDTH_5HZ,
    DLPF_BANDWIDTH_10HZ,
    DLPF_BANDWIDTH_20HZ,
    DLPF_BANDWIDTH_41HZ,
    DLPF_BANDWIDTH_92HZ,
    DLPF_BANDWIDTH_184HZ,
    DLPF_BANDWIDTH_250HZ,
)

# QoS — sensor data: best effort, keep latest only
_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

'''_GYRO_RANGE_MAP = {
    '250':  GYRO_RANGE_250DPS,
    '500':  GYRO_RANGE_500DPS,
    '1000': GYRO_RANGE_1000DPS,
    '2000': GYRO_RANGE_2000DPS,
}

_ACCEL_RANGE_MAP = {
    '2':  ACCEL_RANGE_2G,
    '4':  ACCEL_RANGE_4G,
    '8':  ACCEL_RANGE_8G,
    '16': ACCEL_RANGE_16G,
}'''

_GYRO_RANGE_MAP = {
    250:  GYRO_RANGE_250DPS,
    500:  GYRO_RANGE_500DPS,
    1000: GYRO_RANGE_1000DPS,
    2000: GYRO_RANGE_2000DPS,
}

_ACCEL_RANGE_MAP = {
    2:  ACCEL_RANGE_2G,
    4:  ACCEL_RANGE_4G,
    8:  ACCEL_RANGE_8G,
    16: ACCEL_RANGE_16G,
}
_DLPF_MAP = {
    5:   DLPF_BANDWIDTH_5HZ,
    10:  DLPF_BANDWIDTH_10HZ,
    20:  DLPF_BANDWIDTH_20HZ,
    41:  DLPF_BANDWIDTH_41HZ,
    92:  DLPF_BANDWIDTH_92HZ,
    184: DLPF_BANDWIDTH_184HZ,
    250: DLPF_BANDWIDTH_250HZ,
}

# Covariance matrices — diagonal with empirical noise values
# These are reasonable defaults for the MPU9250; tune via IMU calibration
_GYRO_COVAR  = [1e-4, 0.0, 0.0,
                0.0, 1e-4, 0.0,
                0.0, 0.0, 1e-4]

_ACCEL_COVAR = [1e-2, 0.0, 0.0,
                0.0, 1e-2, 0.0,
                0.0, 0.0, 1e-2]

_MAG_COVAR   = [1e-6, 0.0, 0.0,
                0.0, 1e-6, 0.0,
                0.0, 0.0, 1e-6]

# orientation_covariance = -1 means orientation not provided
_ORIENT_COVAR = [-1.0] + [0.0] * 8


class IMUNode(Node):

    def __init__(self):
        super().__init__('imu_node')

        # ── Declare parameters ────────────────────────────────────────────────
        self.declare_parameter('i2c_bus',        1)
        self.declare_parameter('i2c_address',    0x68)
        self.declare_parameter('publish_rate',   100)
        self.declare_parameter('frame_id',       'imu_link')
        self.declare_parameter('gyro_range',     500)
        self.declare_parameter('accel_range',    4)
        self.declare_parameter('dlpf_bw',        41)
        self.declare_parameter('calibrate_gyro', True)
        self.declare_parameter('calibrate_accel',False)
        # Pre-loaded calibration biases (set these after running calibration)
        self.declare_parameter('cal_gyro_bias_x',  0.0)
        self.declare_parameter('cal_gyro_bias_y',  0.0)
        self.declare_parameter('cal_gyro_bias_z',  0.0)
        self.declare_parameter('cal_accel_bias_x', 0.0)
        self.declare_parameter('cal_accel_bias_y', 0.0)
        self.declare_parameter('cal_accel_bias_z', 0.0)

        # ── Publishers ────────────────────────────────────────────────────────
        self.pub_imu  = self.create_publisher(Imu,          '/imu/data_raw', _SENSOR_QOS)
        self.pub_mag  = self.create_publisher(MagneticField, '/imu/mag',     _SENSOR_QOS)
        self.pub_temp = self.create_publisher(Temperature,   '/imu/temperature', _SENSOR_QOS)

        # ── Calibration services ──────────────────────────────────────────────
        self.create_service(Trigger, '/imu/calibrate_gyro',  self._svc_cal_gyro)
        self.create_service(Trigger, '/imu/calibrate_accel', self._svc_cal_accel)

        # ── Initialise driver ─────────────────────────────────────────────────
        self._init_driver()

        # ── Publish timer ─────────────────────────────────────────────────────
        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self._publish_callback)
        self.get_logger().info(f'IMU node ready — publishing at {rate} Hz')

    def _init_driver(self):
        bus     = self.get_parameter('i2c_bus').value
        addr    = self.get_parameter('i2c_address').value
        g_range = _GYRO_RANGE_MAP.get(
            self.get_parameter('gyro_range').value, GYRO_RANGE_500DPS
        )
        a_range = _ACCEL_RANGE_MAP.get(
            self.get_parameter('accel_range').value, ACCEL_RANGE_4G
        )
        dlpf    = _DLPF_MAP.get(
            self.get_parameter('dlpf_bw').value, DLPF_BANDWIDTH_41HZ
        )

        self.get_logger().info(f'Initialising MPU9250 on I2C bus {bus} @ 0x{addr:02X}')
        self._driver = MPU9250Driver(
            bus=bus, address=addr,
            gyro_range=g_range, accel_range=a_range,
            dlpf_bandwidth=dlpf,
        )
        self._driver.initialize()

        # Load pre-saved calibration biases from parameters
        cal = CalibrationData(
            gyro_bias_x  = self.get_parameter('cal_gyro_bias_x').value,
            gyro_bias_y  = self.get_parameter('cal_gyro_bias_y').value,
            gyro_bias_z  = self.get_parameter('cal_gyro_bias_z').value,
            accel_bias_x = self.get_parameter('cal_accel_bias_x').value,
            accel_bias_y = self.get_parameter('cal_accel_bias_y').value,
            accel_bias_z = self.get_parameter('cal_accel_bias_z').value,
        )
        # Factory mag sensitivity already set in driver.initialize()
        cal.mag_scale_x = self._driver.get_calibration().mag_scale_x
        cal.mag_scale_y = self._driver.get_calibration().mag_scale_y
        cal.mag_scale_z = self._driver.get_calibration().mag_scale_z
        self._driver.set_calibration(cal)

        # Run startup calibration if requested
        if self.get_parameter('calibrate_gyro').value:
            self.get_logger().info('Running gyro calibration — keep sensor still...')
            self._driver.calibrate_gyro()
            c = self._driver.get_calibration()
            self.get_logger().info(
                f'Gyro bias: x={c.gyro_bias_x:.5f} '
                f'y={c.gyro_bias_y:.5f} z={c.gyro_bias_z:.5f} rad/s'
            )

        if self.get_parameter('calibrate_accel').value:
            self.get_logger().info('Running accel calibration — keep sensor flat...')
            self._driver.calibrate_accel()

    # ── Publish callback ──────────────────────────────────────────────────────

    def _publish_callback(self):
        try:
            reading = self._driver.read()
        except Exception as e:
            self.get_logger().warn(f'IMU read failed: {e}', throttle_duration_sec=5.0)
            return

        stamp    = self.get_clock().now().to_msg()
        frame_id = self.get_parameter('frame_id').value
        header   = Header(stamp=stamp, frame_id=frame_id)

        # ── Imu message ───────────────────────────────────────────────────────
        imu_msg = Imu()
        imu_msg.header = header

        # Orientation unknown — set covariance[0] = -1 to signal this
        imu_msg.orientation_covariance = _ORIENT_COVAR

        imu_msg.angular_velocity.x = reading.gyro_x
        imu_msg.angular_velocity.y = reading.gyro_y
        imu_msg.angular_velocity.z = reading.gyro_z
        imu_msg.angular_velocity_covariance = _GYRO_COVAR

        imu_msg.linear_acceleration.x = reading.accel_x
        imu_msg.linear_acceleration.y = reading.accel_y
        imu_msg.linear_acceleration.z = reading.accel_z
        imu_msg.linear_acceleration_covariance = _ACCEL_COVAR

        self.pub_imu.publish(imu_msg)

        # ── MagneticField message ─────────────────────────────────────────────
        if reading.mag_valid:
            mag_msg = MagneticField()
            mag_msg.header = header
            # Convert µT → Tesla for ROS standard
            mag_msg.magnetic_field.x = reading.mag_x * 1e-6
            mag_msg.magnetic_field.y = reading.mag_y * 1e-6
            mag_msg.magnetic_field.z = reading.mag_z * 1e-6
            mag_msg.magnetic_field_covariance = _MAG_COVAR
            self.pub_mag.publish(mag_msg)

        # ── Temperature message ───────────────────────────────────────────────
        temp_msg = Temperature()
        temp_msg.header   = header
        temp_msg.temperature = reading.temperature
        temp_msg.variance    = 0.0
        self.pub_temp.publish(temp_msg)

    # ── Calibration services ──────────────────────────────────────────────────

    def _svc_cal_gyro(self, request, response):
        try:
            self.get_logger().info('Gyro calibration requested — keep still...')
            self._driver.calibrate_gyro()
            c = self._driver.get_calibration()
            response.success = True
            response.message = (
                f'Gyro bias: x={c.gyro_bias_x:.5f} '
                f'y={c.gyro_bias_y:.5f} z={c.gyro_bias_z:.5f} rad/s'
            )
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _svc_cal_accel(self, request, response):
        try:
            self.get_logger().info('Accel calibration requested — keep flat...')
            self._driver.calibrate_accel()
            c = self._driver.get_calibration()
            response.success = True
            response.message = (
                f'Accel bias: x={c.accel_bias_x:.5f} '
                f'y={c.accel_bias_y:.5f} z={c.accel_bias_z:.5f} m/s²'
            )
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def destroy_node(self):
        self._driver.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
