"""Simulated UGV1: Gazebo Harmonic, sensor bridge, and the usual topic names.

Run on the simulation machine (needs ros-jazzy-ros-gz-sim and ros-jazzy-ros-gz-bridge):

  ros2 launch ugv1_controller sim.launch.py                # headless, web page on :8080
  ros2 launch ugv1_controller sim.launch.py gui:=true      # with the Gazebo window
  ros2 launch ugv1_controller sim.launch.py route:=true    # drive the scripted route

Publishes /imu/data_raw, /video_raw_raw, /video_raw, /scan, /clock, /wheel_odom
and /ground_truth/odom, and listens on /cmd_vel. Static transforms are the same
as lidar.launch.py (base_link -> imu_link, laser).
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _static_tf(parent, child, x, y, z, yaw):
    return Node(
        package='tf2_ros', executable='static_transform_publisher',
        name=f'{parent}_to_{child}', output='log',
        arguments=['--x', x, '--y', y, '--z', z, '--yaw', yaw,
                   '--frame-id', parent, '--child-frame-id', child],
        parameters=[{'use_sim_time': True}])


def generate_launch_description():
    share = get_package_share_directory('ugv1_controller')
    models = os.path.join(share, 'sim', 'models')
    world = os.path.join(share, 'sim', 'worlds', 'warehouse.sdf')
    bridge_cfg = os.path.join(share, 'config', 'sim', 'bridge.yaml')
    sim_time = {'use_sim_time': True}

    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = models + (os.pathsep + existing if existing else '')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false',
                              description='Open the Gazebo window (slow without a GPU)'),
        DeclareLaunchArgument('web', default_value='true',
                              description='Start web_node: /drive page and camera feed'),
        DeclareLaunchArgument('route', default_value='false',
                              description='Drive the scripted route instead of the web page'),
        DeclareLaunchArgument('route_speed', default_value='0.3'),

        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        # Software rendering is the realistic case in a VM
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', os.environ.get('LIBGL_ALWAYS_SOFTWARE', '1')),

        # Simulator: server only (and headless rendering) unless the GUI is wanted
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-s', '--headless-rendering', world],
            output='screen', condition=UnlessCondition(LaunchConfiguration('gui'))),
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world],
            output='screen', condition=IfCondition(LaunchConfiguration('gui'))),

        Node(package='ros_gz_bridge', executable='parameter_bridge', name='sim_bridge',
             output='screen', parameters=[{'config_file': bridge_cfg}]),

        _static_tf('base_link', 'imu_link', '0', '0', '0', str(math.pi)),
        _static_tf('base_link', 'laser', '-0.0835', '-0.0063', '0.0947', '3.14159'),

        Node(package='ugv1_controller', executable='sim_camera_relay', output='log',
             parameters=[sim_time]),
        Node(package='ugv1_controller', executable='web_node', output='screen',
             parameters=[sim_time], condition=IfCondition(LaunchConfiguration('web'))),
        Node(package='ugv1_controller', executable='sim_route_node', output='screen',
             parameters=[sim_time, {'speed': LaunchConfiguration('route_speed')}],
             condition=IfCondition(LaunchConfiguration('route'))),
    ])
