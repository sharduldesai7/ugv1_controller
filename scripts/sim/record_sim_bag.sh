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
# NO_SIM_TIME=1 records with wall-clock stamps instead (fallback). The bag then plays back
# slower than the sensors ran; replay with RATE set to simulated seconds per wall second
# (for example RATE=1.28 when the simulation runs at 0.78x), see replay_on_pi.sh.
SIMFLAG="--use-sim-time"; [ -n "$NO_SIM_TIME" ] && SIMFLAG=""
# --foreground and no stdin: without them the recorder (which reads the keyboard) is stopped
# by the shell as a background job and never records anything.
timeout --foreground -s INT -k 120 "$DURATION" ros2 bag record -o "$OUT" $SIMFLAG \
  --compression-mode file --compression-format zstd \
  --topics /imu/data_raw /video_raw_raw /scan /ground_truth/odom /wheel_odom < /dev/null || true
echo "Stopped. Contents:"; ls -lh "$OUT"; ros2 bag info "$OUT" | head -20
