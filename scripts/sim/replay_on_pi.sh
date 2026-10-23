#!/bin/bash
# Replay a simulation bag into OpenVINS on the Raspberry Pi at real-time speed,
# and record what OpenVINS produces plus Pi CPU use.
#
#   ./replay_on_pi.sh <sim_bag_dir> [out_name]
#   RATE=1.28 ./replay_on_pi.sh ...    # only for bags recorded with NO_SIM_TIME=1, see record_sim_bag.sh
#
# Copy the bag to the Pi first (scp -r). Stop the real sensor nodes first
# (tmux kill-server): the replay publishes the same topic names.
#
# Output in ~/sim_runs/<out_name>/:
#   openvins_out/   bag with /odomimu, /odom, /imu/data_raw (for scoring)
#   cpu.csv         one line per second: time, OpenVINS %CPU, RSS MB, system load
# Score afterwards with:
#   ros2 run ugv1_controller vio_eval --truth <sim_bag_dir> --est ~/sim_runs/<out_name>/openvins_out

set -e
BAG="$1"
[ -d "$BAG" ] || { echo "Usage: $0 <sim_bag_dir> [out_name]"; exit 1; }
NAME="${2:-run_$(date +%Y%m%d_%H%M)}"
OUT="$HOME/sim_runs/$NAME"
CFG="$HOME/ros2_ws/src/ugv1_controller/config/sim/estimator_config.yaml"
[ -f "$CFG" ] || { echo "Missing $CFG (git pull and colcon build first)"; exit 1; }
mkdir -p "$OUT"

# Config experiments without touching the repo: OVERRIDES="try_zupt=false max_clones=12" replay_on_pi.sh ...
# copies config/sim to the run folder and changes those top-level keys there.
if [ -n "${OVERRIDES:-}" ]; then
  rm -rf "$OUT/config" && cp -r "$(dirname "$CFG")" "$OUT/config"
  CFG="$OUT/config/estimator_config.yaml"
  for kv in $OVERRIDES; do
    k="${kv%%=*}"; v="${kv#*=}"
    grep -q "^$k:" "$CFG" || { echo "Override key '$k' not found in $CFG"; exit 1; }
    sed -i "s|^$k:.*|$k: $v|" "$CFG"
    echo "override: $k = $v"
  done
fi
source ~/ros2_ws/install/setup.bash

tmux kill-session -t sim_ov 2>/dev/null || true
tmux kill-session -t sim_odom_tf 2>/dev/null || true
tmux kill-session -t sim_rec 2>/dev/null || true

tmux new-session -d -s sim_ov "source ~/ros2_ws/install/setup.bash; ros2 run ov_msckf run_subscribe_msckf --ros-args -p config_path:=$CFG 2>&1 | tee $OUT/openvins.log; echo OPENVINS EXITED >> $OUT/openvins.log; sleep 600"
tmux new-session -d -s sim_odom_tf "source ~/ros2_ws/install/setup.bash; ros2 run ugv1_controller odom_tf_node"
sleep 5
rm -rf "$OUT/openvins_out"
tmux new-session -d -s sim_rec "source ~/ros2_ws/install/setup.bash; ros2 bag record -o $OUT/openvins_out /odomimu /odom /imu/data_raw"
sleep 2

# CPU logger: the OpenVINS program itself (process name run_subscribe_m, as ps truncates it).
# Matching the command line instead would also hit the tmux shell that launched it.
( echo "time_s,ov_cpu_percent,ov_rss_mb,load1"
  t0=$(date +%s)
  while true; do
    pid=$(pgrep -x run_subscribe_m | head -n1)
    if [ -n "$pid" ]; then
      cpu=$(top -b -n 2 -d 0.5 -p "$pid" | awk -v p="$pid" '$1==p {c=$9} END {print c}')
      rss=$(ps -o rss= -p "$pid" | tr -d ' ')
      echo "$(( $(date +%s) - t0 )),${cpu:-0},$(( ${rss:-0} / 1024 )),$(cut -d' ' -f1 /proc/loadavg)"
    fi
    sleep 0.5
  done ) > "$OUT/cpu.csv" &
LOGGER=$!

echo "Playing $BAG at 1x. Leave the Pi otherwise idle so the CPU figures are meaningful."
ros2 bag play "$BAG" --topics /imu/data_raw /video_raw_raw /scan --rate "${RATE:-1.0}"
sleep 3
kill $LOGGER 2>/dev/null || true
tmux send-keys -t sim_rec C-c
sleep 3
tmux kill-session -t sim_ov 2>/dev/null || true
tmux kill-session -t sim_odom_tf 2>/dev/null || true

echo ""
echo "Done. Results in $OUT"
awk -F, 'NR>1 {n++; s+=$2; if ($2>m) m=$2; if ($3>r) r=$3} END {if (n) printf "OpenVINS CPU: mean %.0f%%, peak %.0f%% (400%% = all four cores); peak memory %d MB\n", s/n, m, r}' "$OUT/cpu.csv"
