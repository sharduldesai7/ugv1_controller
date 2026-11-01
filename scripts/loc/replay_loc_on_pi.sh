#!/bin/bash
# Standard localisation stack (EKF + slam_toolbox): replay a simulation bag into it on the
# Raspberry Pi at real-time speed, record the result, and measure Pi load and dropped scans.
#
#   ./replay_loc_on_pi.sh <sim_bag_dir> [out_name]
#   USE_EKF=false ./replay_loc_on_pi.sh ...     # wheel odometry straight to slam_toolbox
#
# Stop other sensor nodes first (tmux kill-server): the replay publishes the same topic names.
#
# Output in ~/loc_runs/<out_name>/:
#   loc_out/    bag with /localization/map_pose and /localization/odom_pose (for scoring)
#   launch.log  stack output (also counted for dropped scans)
#   cpu.csv     one line per second: time, %CPU and RSS MB per process, load
#   map.posegraph / map.data   serialized map, if the service answered
# Score with:
#   ros2 run ugv1_controller loc_eval --truth <sim_bag_dir> --est ~/loc_runs/<out_name>/loc_out --timeline 30

set -e
BAG="$1"
[ -d "$BAG" ] || { echo "Usage: $0 <sim_bag_dir> [out_name]"; exit 1; }
NAME="${2:-run_$(date +%Y%m%d_%H%M)}"
OUT="$HOME/loc_runs/$NAME"
EKF="${USE_EKF:-true}"
mkdir -p "$OUT"
source ~/ros2_ws/install/setup.bash

for s in loc_stack loc_rec; do tmux kill-session -t $s 2>/dev/null || true; done

tmux new-session -d -s loc_stack "source ~/ros2_ws/install/setup.bash; ros2 launch ugv1_controller loc.launch.py use_sim_time:=true use_ekf:=$EKF 2>&1 | tee $OUT/launch.log; sleep 600"
sleep 15
rm -rf "$OUT/loc_out"
tmux new-session -d -s loc_rec "source ~/ros2_ws/install/setup.bash; ros2 bag record --use-sim-time -o $OUT/loc_out /localization/map_pose /localization/odom_pose"
sleep 2

# CPU logger. ps truncates process names to 15 characters.
PROCS="async_slam_tool ekf_node python3"
( echo "time_s,proc,cpu_percent,rss_mb,load1"
  t0=$(date +%s)
  while true; do
    for p in $PROCS; do
      for pid in $(pgrep -x "$p"); do
        cmd=$(tr '\0' ' ' < /proc/$pid/cmdline 2>/dev/null | cut -c1-60)
        case "$p" in python3) case "$cmd" in *sensor_conditioner*|*map_pose*) ;; *) continue;; esac;; esac
        cpu=$(top -b -n 1 -p "$pid" | awk -v p="$pid" '$1==p {print $9}')
        rss=$(ps -o rss= -p "$pid" | tr -d ' ')
        echo "$(( $(date +%s) - t0 )),$p:$pid,${cpu:-0},$(( ${rss:-0} / 1024 )),$(cut -d' ' -f1 /proc/loadavg)"
      done
    done
    sleep 1
  done ) > "$OUT/cpu.csv" &
LOGGER=$!

echo "Playing $BAG at 1x (EKF=$EKF). Leave the Pi otherwise idle so the CPU figures are meaningful."
ros2 bag play "$BAG" --clock --topics /imu/data_raw /scan /wheel_odom
sleep 3
kill $LOGGER 2>/dev/null || true

# Save the map (best effort; the service type is checked by the call itself)
timeout 60 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$OUT/map'}" > "$OUT/serialize.log" 2>&1 || echo "serialize_map did not answer (see $OUT/serialize.log)"

tmux send-keys -t loc_rec C-c
sleep 3
for s in loc_stack loc_rec; do tmux kill-session -t $s 2>/dev/null || true; done

echo ""
echo "Done. Results in $OUT"
DROPS=$(grep -c "Message Filter dropping message" "$OUT/launch.log" || true)
echo "slam_toolbox 'Message Filter dropping message' lines: $DROPS (scans lost for lack of a transform)"
awk -F, 'NR>1 {split($2,a,":"); n[a[1]]++; s[a[1]]+=$3; if ($3>m[a[1]]) m[a[1]]=$3; if ($4>r[a[1]]) r[a[1]]=$4}
  END {for (k in n) printf "%-16s CPU mean %.0f%%, peak %.0f%%; peak memory %d MB\n", k, s[k]/n[k], m[k], r[k]}' "$OUT/cpu.csv"
echo "(CPU: 400% = all four cores)"
