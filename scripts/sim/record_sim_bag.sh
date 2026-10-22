#!/bin/bash
# Record the simulated sensors and ground truth. Run on the simulation machine
# after `ros2 launch ugv1_controller sim.launch.py route:=true` is up.
#
#   ./record_sim_bag.sh [bag_name]
#
# Stop with Ctrl-C when sim_route_node logs "Route finished". Time in the bag is
# simulated time, so it does not matter if the simulation ran slower than real time.

set -e
NAME="${1:-aisles_$(date +%Y%m%d_%H%M)}"
OUT="$HOME/sim_bags/$NAME"
mkdir -p "$HOME/sim_bags"
source ~/ros2_ws/install/setup.bash

echo "Recording to $OUT (Ctrl-C to stop)"
exec ros2 bag record -o "$OUT" \
  --compression-mode file --compression-format zstd \
  /imu/data_raw /video_raw_raw /scan /ground_truth/odom /wheel_odom
