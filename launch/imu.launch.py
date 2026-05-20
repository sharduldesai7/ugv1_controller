"""
Launch the MPU9250 IMU node.

Usage:
  ros2 launch rover_imu imu.launch.py
  ros2 launch rover_imu imu.launch.py calibrate_gyro:=true
  ros2 launch rover_imu imu.launch.py publish_rate:=200 gyro_range:=250

Persisting calibration biases:
  After running calibration via service call, copy the printed bias values
  here as default_value so they survive restarts:

    cal_gyro_bias_x  default_value='<value from calibration>'
    cal_gyro_bias_y  default_value='<value>'
    cal_gyro_bias_z  default_value='<value>'
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ── Hardware config ────────────────────────────────────────────────────
        DeclareLaunchArgument('i2c_bus',     default_value='1',    description='I2C bus number'),
        DeclareLaunchArgument('i2c_address', default_value='104',  description='MPU9250 I2C address (0x68=104)'),

        # ── Sensor ranges ──────────────────────────────────────────────────────
        DeclareLaunchArgument('gyro_range',  default_value='500',  description='Gyro range dps: 250|500|1000|2000'),
        DeclareLaunchArgument('accel_range', default_value='4',    description='Accel range g: 2|4|8|16'),
        DeclareLaunchArgument('dlpf_bw',     default_value='41',   description='DLPF bandwidth Hz: 5|10|20|41|92|184|250'),

        # ── Publish config ─────────────────────────────────────────────────────
        DeclareLaunchArgument('publish_rate',    default_value='100', description='Publish rate Hz'),
        DeclareLaunchArgument('frame_id',        default_value='imu_link', description='TF frame ID'),

        # ── Calibration ────────────────────────────────────────────────────────
        DeclareLaunchArgument('calibrate_gyro',  default_value='false',  description='Run gyro cal on start'),
        DeclareLaunchArgument('calibrate_accel', default_value='false', description='Run accel cal on start'),

        # ── Pre-saved calibration biases (fill these in after calibration) ─────
        DeclareLaunchArgument('cal_gyro_bias_x',  default_value='-0.09462'),
        DeclareLaunchArgument('cal_gyro_bias_y',  default_value='-0.17138'),
        DeclareLaunchArgument('cal_gyro_bias_z',  default_value='-0.20097'),
        DeclareLaunchArgument('cal_accel_bias_x', default_value='-12.11120'),
        DeclareLaunchArgument('cal_accel_bias_y', default_value='3.55178'),
        DeclareLaunchArgument('cal_accel_bias_z', default_value='-2.27557'),

        # ── IMU node ───────────────────────────────────────────────────────────
        Node(
            #package='rover_imu',
            package='ugv1_controller',
            executable='imu_node',
            name='imu_node',
            output='screen',
            parameters=[{
                'i2c_bus':        LaunchConfiguration('i2c_bus'),
                'i2c_address':    LaunchConfiguration('i2c_address'),
                'publish_rate':   LaunchConfiguration('publish_rate'),
                'frame_id':       LaunchConfiguration('frame_id'),
                'gyro_range':     LaunchConfiguration('gyro_range'),
                'accel_range':    LaunchConfiguration('accel_range'),
                'dlpf_bw':        LaunchConfiguration('dlpf_bw'),
                'calibrate_gyro': LaunchConfiguration('calibrate_gyro'),
                'calibrate_accel':LaunchConfiguration('calibrate_accel'),
                'cal_gyro_bias_x': LaunchConfiguration('cal_gyro_bias_x'),
                'cal_gyro_bias_y': LaunchConfiguration('cal_gyro_bias_y'),
                'cal_gyro_bias_z': LaunchConfiguration('cal_gyro_bias_z'),
                'cal_accel_bias_x':LaunchConfiguration('cal_accel_bias_x'),
                'cal_accel_bias_y':LaunchConfiguration('cal_accel_bias_y'),
                'cal_accel_bias_z':LaunchConfiguration('cal_accel_bias_z'),
            }],
        ),
    ])
