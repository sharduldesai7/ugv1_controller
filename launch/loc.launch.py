"""Standard localisation: wheel odometry + gyro (EKF) and slam_toolbox on /scan.

  ros2 launch ugv1_controller loc.launch.py use_sim_time:=true                  # simulation or bag replay
  ros2 launch ugv1_controller loc.launch.py use_sim_time:=true use_ekf:=false   # wheel odometry straight to slam_toolbox
  ros2 launch ugv1_controller loc.launch.py use_sim_time:=true publish_static_tf:=false   # sim.launch.py already runs

Inputs:  /wheel_odom, /imu/data_raw, /scan
Outputs: TF odom -> base_link (EKF, or the conditioner without it), TF map -> odom (slam_toolbox),
         /map, /localization/map_pose, /localization/odom_pose

publish_static_tf: base_link -> imu_link and laser. Leave true for bag replay; set false when
sim.launch.py or lidar.launch.py is running, since they publish the same transforms.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _static_tf(parent, child, x, y, z, yaw, use_sim_time):
    return Node(
        package='tf2_ros', executable='static_transform_publisher',
        name=f'{parent}_to_{child}', output='log',
        arguments=['--x', x, '--y', y, '--z', z, '--yaw', yaw,
                   '--frame-id', parent, '--child-frame-id', child],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('publish_static_tf')))


def generate_launch_description():
    share = get_package_share_directory('ugv1_controller')
    slam_share = get_package_share_directory('slam_toolbox')
    ekf_cfg = os.path.join(share, 'config', 'loc', 'ekf.yaml')
    slam_cfg = os.path.join(share, 'config', 'loc', 'slam_toolbox_mapping.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ekf = LaunchConfiguration('use_ekf')
    sim = {'use_sim_time': use_sim_time}

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_ekf', default_value='true',
                              description='false: wheel odometry feeds slam_toolbox directly'),
        DeclareLaunchArgument('publish_static_tf', default_value='true'),

        _static_tf('base_link', 'imu_link', '0', '0', '0', str(math.pi), use_sim_time),
        _static_tf('base_link', 'laser', '-0.0835', '-0.0063', '0.0947', '3.14159', use_sim_time),

        Node(package='ugv1_controller', executable='sensor_conditioner_node',
             name='sensor_conditioner', output='screen',
             parameters=[sim, {'publish_tf': PythonExpression(["'", use_ekf, "' == 'false'"])}]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             output='screen', parameters=[ekf_cfg, sim], condition=IfCondition(use_ekf)),
        Node(package='ugv1_controller', executable='map_pose_node', name='map_pose_node',
             output='screen', parameters=[sim]),
        # slam_toolbox's own launch file also handles its lifecycle start-up.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_share, 'launch', 'online_async_launch.py')),
            launch_arguments={'slam_params_file': slam_cfg,
                              'use_sim_time': use_sim_time,
                              'autostart': 'true'}.items()),
    ])
