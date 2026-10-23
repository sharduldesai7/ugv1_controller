#!/bin/bash
# Record the simulated sensors and ground truth. Run on the simulation machine
# AFTER `ros2 launch ugv1_controller sim.launch.py route:=true` is up (the rover stands still for
# 30 s of simulated time first, so there is time to start this; --use-sim-time needs the clock).
#
#   ./record_sim_bag.sh [bag_name]
#
# The bag is stamped with simulated time (--use-sim-time), so replaying it at 1x gives the Pi the
# true sensor rates even though the simulation itself runs slower than real time.
#
# Stops by itself after DURATION wall-clock seconds (default 600: the route needs about 250 s of
# simulated time, i.e. 5-8 minutes of wall time depending on how fast the simulation runs).
# Ctrl-C also stops it cleanly; do not close the terminal or put the laptop to sleep while recording.
#   DURATION=900 ./record_sim_bag.sh my_bag

set -e
NAME="${1:-aisles_$(date +%Y%m%d_%H%M)}"
OUT="$HOME/sim_bags/$NAME"
mkdir -p "$HOME/sim_bags"
source ~/ros2_ws/install/setup.bash

DURATION="${DURATION:-600}"
echo "Recording to $OUT for up to ${DURATION} s (Ctrl-C to stop earlier)"
timeout -s INT -k 120 "$DURATION" ros2 bag record -o "$OUT" --use-sim-time \
  --compression-mode file --compression-format zstd \
  /imu/data_raw /video_raw_raw /scan /ground_truth/odom /wheel_odom || true
echo "Stopped. Contents:"; ls -lh "$OUT"; ros2 bag info "$OUT" | head -20
