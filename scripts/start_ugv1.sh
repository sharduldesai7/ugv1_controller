#!/bin/bash
# Starts all UGV1 nodes in the correct order
#
# Usage:
#   ./start_ugv1.sh [--with-openvins] [--peer <ip>]
#
# Options:
#   --with-openvins   Also start the OpenVINS VIO node
#   --peer <ip>       IP address of a remote ROS2 peer (e.g. the visualization
#                      VM) to enable cross-machine topic discovery via
#                      ROS_STATIC_PEERS. Omit this flag to run Pi-only with
#                      default (local) discovery settings.

set -e

WITH_OPENVINS=false
PEER_IP=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-openvins)
      WITH_OPENVINS=true
      shift
      ;;
    --peer)
      PEER_IP="$2"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1"
      echo "Usage: $0 [--with-openvins] [--peer <ip>]"
      exit 1
      ;;
  esac
done

source ~/ros2_ws/install/setup.bash

# Build the env var prefix used inside each tmux session.
# If --peer was given, enable static-peer discovery so a remote machine
# (e.g. a visualization VM) on the same subnet can see these topics.
DISCOVERY_ENV=""
if [ -n "$PEER_IP" ]; then
  DISCOVERY_ENV="export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET; export ROS_STATIC_PEERS=${PEER_IP};"
  echo "Static peer discovery enabled — advertising to peer at ${PEER_IP}"
else
  echo "No --peer specified — running with local discovery only"
fi

echo "Starting UGV1 stack..."

# Start IMU node with calibration biases
tmux new-session -d -s ugv1_imu \
  "${DISCOVERY_ENV} source ~/ros2_ws/install/setup.bash; ros2 run ugv1_controller imu_node --ros-args \
    -p i2c_address:=105 \
    -p calibrate_gyro:=false \
    -p calibrate_accel:=false \
    -p cal_gyro_bias_x:=0.0 \
    -p cal_gyro_bias_y:=0.0 \
    -p cal_gyro_bias_z:=0.0 \
    -p cal_accel_bias_x:=0.0 \
    -p cal_accel_bias_y:=0.0 \
    -p cal_accel_bias_z:=0.0"

echo "IMU node started."
sleep 2

# Start camera node with raw publishing enabled (needed for OpenVINS)
tmux new-session -d -s ugv1_camera \
  "${DISCOVERY_ENV} source ~/ros2_ws/install/setup.bash; ros2 run ugv1_controller capture_node --ros-args -p publish_raw:=true"

echo "Camera node started."
sleep 2

# Start motor node
tmux new-session -d -s ugv1_motor \
  "${DISCOVERY_ENV} source ~/ros2_ws/install/setup.bash; ros2 run ugv1_controller motor_node"

echo "Motor node started."
sleep 1

# Start the web server (landing page at / and camera viewer at /processed-feed)
tmux new-session -d -s ugv1_viewer \
  "${DISCOVERY_ENV} source ~/ros2_ws/install/setup.bash; ros2 run ugv1_controller viewer_node"

echo "Viewer node started — http://<pi-ip>:8080"
sleep 1

# Optionally start OpenVINS
if [ "$WITH_OPENVINS" = true ]; then
  echo "Starting OpenVINS... Hold robot still for initialization."
  tmux new-session -d -s ugv1_openvins \
    "${DISCOVERY_ENV} source ~/ros2_ws/install/setup.bash; ros2 run ov_msckf run_subscribe_msckf --ros-args \
      -p config_path:=/home/ubuntu/ros2_ws/src/ugv1_controller/config/estimator_config.yaml"
  echo "OpenVINS started."
fi

echo ""
echo "UGV1 stack is running. Attach to sessions with:"
echo "  tmux attach -t ugv1_imu"
echo "  tmux attach -t ugv1_camera"
echo "  tmux attach -t ugv1_motor"
echo "  tmux attach -t ugv1_viewer"
[ "$WITH_OPENVINS" = true ] && echo "  tmux attach -t ugv1_openvins"
echo ""
echo "Stop all with: tmux kill-server"
