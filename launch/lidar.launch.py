"""
Launch the RPLidar A1M8 driver and the fixed transforms for the robot body.

Usage:
  ros2 launch ugv1_controller lidar.launch.py
  ros2 launch ugv1_controller lidar.launch.py serial_port:=/dev/ttyUSB1
  ros2 launch ugv1_controller lidar.launch.py lidar_yaw:=3.14159

Frames (ROS convention, REP-103: x forward, y left, z up):
  base_link   robot body frame, placed at the IMU
  imu_link    the IMU's own axes. The IMU x axis points BACKWARD (z up), so it
              is base_link rotated 180 degrees about z.
  laser       the LiDAR. Fixed on the centreline, 15 cm behind the camera and
              8 cm above it. The camera is 6.65 cm in front of the IMU and
              1.47 cm above it (Kalibr result), so in base_link the LiDAR is
              behind and above the origin:
                x = -(0.15 - 0.0665) = -0.0835 m
                y = -0.0063 m  (centreline offset of the camera, assumed)
                z = 0.0147 + 0.08 = 0.0947 m

The LiDAR offsets were derived from measurements; replace them with values
measured on the finished robot via the lidar_x / lidar_y / lidar_z arguments.
lidar_yaw rotates the laser frame about z: set it so that the scan's forward
direction in RViz matches the robot's real front.
"""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _static_tf(parent, child, x, y, z, yaw, name):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            '--x', x, '--y', y, '--z', z,
            '--yaw', yaw, '--pitch', '0', '--roll', '0',
            '--frame-id', parent, '--child-frame-id', child,
        ],
    )


def generate_launch_description():
    return LaunchDescription([

        # ── Driver config ──────────────────────────────────────────────────────
        # The udev rule in scripts/99-ugv1-usb.rules creates /dev/rplidar. Fall
        # back to /dev/ttyUSB0 only if you have not installed it (the motor
        # controller's USB adapter may also claim that name).
        DeclareLaunchArgument('serial_port',     default_value='/dev/rplidar'),
        DeclareLaunchArgument('serial_baudrate', default_value='115200',
                              description='A1M8 uses 115200'),
        DeclareLaunchArgument('frame_id',        default_value='laser'),

        # ── LiDAR pose in base_link (metres / radians) ─────────────────────────
        DeclareLaunchArgument('lidar_x',   default_value='-0.0835'),
        DeclareLaunchArgument('lidar_y',   default_value='-0.0063'),
        DeclareLaunchArgument('lidar_z',   default_value='0.0947'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'channel_type':     'serial',
                'serial_port':      LaunchConfiguration('serial_port'),
                'serial_baudrate':  LaunchConfiguration('serial_baudrate'),
                'frame_id':         LaunchConfiguration('frame_id'),
                'inverted':         False,
                'angle_compensate': True,
            }],
        ),

        # base_link -> imu_link: IMU x points backward, so rotate 180 deg about z
        _static_tf('base_link', 'imu_link', '0', '0', '0',
                   str(math.pi), 'base_link_to_imu_link'),

        # base_link -> laser
        _static_tf('base_link', LaunchConfiguration('frame_id'),
                   LaunchConfiguration('lidar_x'),
                   LaunchConfiguration('lidar_y'),
                   LaunchConfiguration('lidar_z'),
                   LaunchConfiguration('lidar_yaw'),
                   'base_link_to_laser'),
    ])
