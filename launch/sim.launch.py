"""Simulated UGV1: Gazebo Harmonic, sensor bridge, and the usual topic names.

Run on the simulation machine (needs ros-jazzy-ros-gz-sim and ros-jazzy-ros-gz-bridge):

  ros2 launch ugv1_controller sim.launch.py                # headless, web page on :8080
  ros2 launch ugv1_controller sim.launch.py gui:=true      # with the Gazebo window
  ros2 launch ugv1_controller sim.launch.py route:=true    # drive the scripted route

Rendering (the cameras and LiDAR are rendered, so Gazebo needs an OpenGL context):
  render:=egl    headless EGL, no forced software rendering (default)
  render:=xvfb   virtual X display with software OpenGL (needs: sudo apt install xvfb)
  render:=soft-egl  headless EGL with software rendering forced (crashes in some VMs)

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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _static_tf(parent, child, x, y, z, yaw):
    return Node(
        package='tf2_ros', executable='static_transform_publisher',
        name=f'{parent}_to_{child}', output='log',
        arguments=['--x', x, '--y', y, '--z', z, '--yaw', yaw,
                   '--frame-id', parent, '--child-frame-id', child],
        parameters=[{'use_sim_time': True}])


def _both(mode):
    """True when the GUI is off and render equals `mode`."""
    return PythonExpression(["'", LaunchConfiguration('gui'), "' == 'false' and '",
                             LaunchConfiguration('render'), "' == '", mode, "'"])


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
        DeclareLaunchArgument('render', default_value='egl',
                              description='egl | xvfb | soft-egl (see the file header)'),

        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        # Simulator. With the GUI, a normal run; otherwise server only, in one of three
        # rendering setups (the default needs no extra packages).
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world],
            output='screen', condition=IfCondition(LaunchConfiguration('gui'))),
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-s', '--headless-rendering', world],
            output='screen',
            condition=IfCondition(_both('egl'))),
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-s', '--headless-rendering', world],
            additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'}, output='screen',
            condition=IfCondition(_both('soft-egl'))),
        ExecuteProcess(
            cmd=['xvfb-run', '-a', '-s', '-screen 0 1280x1024x24', 'gz', 'sim', '-r', '-s', world],
            additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'}, output='screen',
            condition=IfCondition(_both('xvfb'))),

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
