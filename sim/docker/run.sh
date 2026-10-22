#!/bin/bash
# Build and start the simulation container (run on the Mac, from anywhere).
#
#   sim/docker/run.sh            # build if needed, start, print how to connect
#   docker exec -it ugv1-sim bash
#
# Mounts: this repository -> /root/ros2_ws/src/ugv1_controller, ~/sim_bags -> /root/sim_bags
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
mkdir -p "$HOME/sim_bags"

docker build -t ugv1-sim "$HERE"
docker rm -f ugv1-sim 2>/dev/null || true
docker run -d --name ugv1-sim --shm-size=1g \
  -p 5901:5901 -p 8080:8080 \
  -v "$REPO":/root/ros2_ws/src/ugv1_controller \
  -v "$HOME/sim_bags":/root/sim_bags \
  ugv1-sim

cat <<MSG

Container 'ugv1-sim' is up.
  Desktop:  Finder > Go > Connect to Server > vnc://localhost:5901   (password: password)
  Shell:    docker exec -it ugv1-sim bash
  First time inside the shell:
    cd /root/ros2_ws && colcon build --packages-select ugv1_controller --symlink-install && source install/setup.bash
  Then:
    ros2 launch ugv1_controller sim.launch.py render:=xvfb          # no window; web page at http://localhost:8080/drive
    ros2 launch ugv1_controller sim.launch.py gui:=true             # Gazebo window on the VNC desktop
  Bags land in ~/sim_bags on the Mac (record with: scripts/sim/record_sim_bag.sh, see sim/README.md).
MSG
