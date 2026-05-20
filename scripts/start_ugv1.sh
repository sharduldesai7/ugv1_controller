#!/bin/bash
# Starts all UGV1 nodes in the correct order
# Usage: ./start_ugv1.sh [--with-openvins]

set -e

source ~/ros2_ws/install/setup.bash

echo "Starting UGV1 stack..."

# Start IMU node with calibration values
tmux new-session -d -s ugv1_imu \
  "ros2 run ugv1_controller imu_node --ros-args \
    -p calibrate_gyro:=false \
    -p calibrate_accel:=false \
    -p cal_gyro_bias_x:=-0.09462 \
    -p cal_gyro_bias_y:=-0.17138 \
    -p cal_gyro_bias_z:=-0.20097 \
    -p cal_accel_bias_x:=-12.11120 \
    -p cal_accel_bias_y:=3.55178 \
    -p cal_accel_bias_z:=-2.27557"

echo "IMU node started."
sleep 2

# Start camera node with raw publishing enabled
tmux new-session -d -s ugv1_camera \
  "ros2 run ugv1_controller capture_node --ros-args -p publish_raw:=true"

echo "Camera node started."
sleep 2

# Start motor node
tmux new-session -d -s ugv1_motor \
  "ros2 run ugv1_controller motor_node"

echo "Motor node started."
sleep 1

# Optionally start OpenVINS
if [ "$1" == "--with-openvins" ]; then
  echo "Starting OpenVINS... Hold robot still for initialization."
  tmux new-session -d -s ugv1_openvins \
    "ros2 run ov_msckf run_subscribe_msckf --ros-args \
      -p config_path:=/home/ubuntu/ros2_ws/src/ugv1_controller/config/estimator_config.yaml"
  echo "OpenVINS started."
fi

echo ""
echo "UGV1 stack is running. Attach to sessions with:"
echo "  tmux attach -t ugv1_imu"
echo "  tmux attach -t ugv1_camera"
echo "  tmux attach -t ugv1_motor"
[ "$1" == "--with-openvins" ] && echo "  tmux attach -t ugv1_openvins"
echo ""
echo "Stop all with: tmux kill-server"
