# Standard localisation (EKF + slam_toolbox)

LiDAR + IMU + wheel encoders. This is the main localisation track; OpenVINS is parked under `docs/vio/`.

## Data flow

    /wheel_odom, /imu/data_raw
        -> sensor_conditioner_node   (sets covariances, removes gyro z bias at start-up)
        -> /wheel_odom/conditioned, /imu/conditioned
        -> robot_localization ekf_node   -> TF odom -> base_link   (vx and turn rate from wheels, turn rate from gyro)
    /scan -> slam_toolbox (async, mapping) -> TF map -> odom, /map
    map_pose_node -> /localization/map_pose (map -> base_link), /localization/odom_pose (odom -> base_link)

`use_ekf:=false` skips the EKF; the conditioner then publishes odom -> base_link straight from the wheel pose, so the two options can be compared under wheel slip (Phase 2).

## Run in the simulation (container)

    sudo apt install ros-jazzy-slam-toolbox ros-jazzy-robot-localization   # once
    ros2 launch ugv1_controller sim.launch.py route:=true
    ros2 launch ugv1_controller loc.launch.py use_sim_time:=true publish_static_tf:=false

## Replay a recorded bag on the Pi

    scripts/loc/replay_loc_on_pi.sh ~/sim_bags/aisles_real run1
    ros2 run ugv1_controller loc_eval --truth ~/sim_bags/aisles_real --est ~/loc_runs/run1/loc_out --timeline 30

The script reports slam_toolbox "Message Filter dropping message" counts, CPU and memory per process, and saves the map. Replay needs `ros2 bag play --clock` and `use_sim_time:=true`, which the script and launch file set.

## Scoring

`loc_eval` aligns the estimate to ground truth by one rotation and translation (no scale), then reports planar error: RMSE, median, 95th percentile, max, final error as a percentage of distance. It also scores the odometry-only pose, which shows how much the map correction removes.

## Status and caveats

- Phase 1 (this): mapping mode on replayed bags. Untested until first run on the Pi or container.
- Proposed acceptance (not yet agreed): under 5% skipped scans and no gap over 0.5 s, measured on the Pi.
- slam_toolbox is started through its own `online_async_launch.py`. Verify it exists: `ls /opt/ros/jazzy/share/slam_toolbox/launch/`.
- The `serialize_map` service type (`slam_toolbox/srv/SerializePoseGraph`) is unverified.
- The sim wheel odometry is ideal (no slip). Phase 2 adds a degradation node and a restocked-world test.
