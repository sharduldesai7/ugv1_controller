# UGV1 Controller

ROS2 Jazzy package for UGV1 — a payload-agnostic autonomous ground vehicle targeting controlled indoor environments.

## Hardware

- Raspberry Pi 4 (4GB) — Ubuntu 24.04 LTS
- OV5647 camera (RPi Camera Module 1) — `--vflip --hflip`
- MPU9250 IMU via I2C at 0x68 (accel + gyro only, magnetometer unavailable)
- Arduino Nano + FT232RL USB-to-serial for motor control
- L293D H-bridge (4 motors)

## ROS2 Nodes

- `capture_node` — captures MJPEG from rpicam-vid, publishes `/video_raw` (CompressedImage). Run with `-p publish_raw:=true` to also publish `/video_raw_raw` (Image) for OpenVINS.
- `imu_node` — raw MPU9250 driver, publishes `/imu/data_raw` at 100Hz
- `motor_control_node` — subscribes to `/cmd_vel`, sends UART packets to Arduino
- `process_node` — OpenCV processing of `/video_raw`, publishes `/video_processed`. Not used by the web interface or started by `start_ugv1.sh`; pending removal (see `TECH_DEBT.md`)
- `web_node` — web server on port 8080 and manual-drive source for `/cmd_vel` (see below)
- `odom_tf_node` — turns OpenVINS `/odomimu` (frames `global` -> `imu`) into the `odom -> base_link` transform and an `/odom` pose topic. base_link is the IMU frame turned 180 deg about z. Started with `--with-openvins` in the `ugv1_odom_tf` session.
- `lidar.launch.py` — RPLidar A1M8 driver (`/scan`) plus the static transforms `base_link -> imu_link` and `base_link -> laser`. Started by `start_ugv1.sh` in the `ugv1_lidar` tmux session.

Serial devices use stable names from `scripts/99-ugv1-usb.rules` (`/dev/rplidar`, `/dev/ugv_motor`); install the rules on the Pi before running the stack.

## Simulation

`sim/` holds a Gazebo Harmonic model of the rover and a textured warehouse, for testing OpenVINS (and later RTAB-Map and Nav2) without the real chassis: record a bag in the simulator, replay it on the Pi at real time, and score the result against ground truth. See `sim/README.md`.

## LiDAR health check

`scripts/lidar_health_check.py` talks to the RPLidar directly over serial (no ROS) and reports whether it answers. Run it before launching the driver, and use it to hunt for a loose connector by flexing the cables while it runs:

```bash
python3 scripts/lidar_health_check.py                        # 10 s check on /dev/rplidar
python3 scripts/lidar_health_check.py --duration 40 --motor  # longer, with the motor spinning
```

Exit code 0 means every request was answered, 1 means intermittent replies, 2 means no reply, 3 means the port could not be opened. Stop the driver first, since only one program can use the port.

## Web interface

`web_node` serves two Flask blueprints from `ugv1_controller/web/` (HTML, CSS and JS are plain files under `web/static/`):

| Path | Description |
| --- | --- |
| `/` | Landing page with project overview and hardware/software architecture diagrams |
| `/drive` | First-person drive page: raw camera feed with a heads-up display, drive controls |
| `/drive/feed/raw` | Raw MJPEG stream |
| `/drive/api/state`, `enable`, `disable`, `cmd`, `speed` | Control API used by the page |
| `/status` | JSON health check (camera frame age, drive state) |

### Driving from the browser

Open `http://<pi-ip>:8080/drive` on a desktop or phone. Nothing moves until **Drive Enable** is switched on.

- **Desktop:** the keys of `teleop_twist_keyboard` (without the Shift sideways keys): `u i o / j k l / m , .` to move and turn, `k` to stop, `q z` both speeds +/- 10%, `w x` linear only, `e c` turn only. Hold a key to drive; release to stop. The same keys are on screen as buttons.
- **Touch screens:** a joystick (bottom right: up forward, down reverse, left/right turn), plus Drive Enable, STOP and the speed buttons on the left.
- **Drive Enable** belongs to one device at a time. It can only be switched off when no motion command is active and the IMU shows the rover at rest for 0.5 s; with no IMU data it refuses. A dropped connection or page reload releases it.
- `web_node` publishes `/cmd_vel` at 20 Hz only while Drive Enable is on, so Nav2 can own `/cmd_vel` when it is off.
- **Safety:** a command older than 0.3 s counts as zero, and `motor_control_node` stops the motors if `/cmd_vel` goes quiet for 0.5 s (`cmd_timeout` parameter, 0 disables). Anything else that drives the rover must publish faster than that.
- Thresholds for "at rest" are parameters of `web_node` (`still_gyro_range` in rad/s, `still_accel_range` in m/s^2). Tune them from the `imu_gyro_range` and `imu_accel_range` values in `/status` with the rover resting.

## Running the stack

### IMU node (with calibration values)
```bash
source ~/ros2_ws/install/setup.bash
ros2 run ugv1_controller imu_node --ros-args \
  -p calibrate_gyro:=false \
  -p calibrate_accel:=false \
  -p cal_gyro_bias_x:=-0.09462 \
  -p cal_gyro_bias_y:=-0.17138 \
  -p cal_gyro_bias_z:=-0.20097 \
  -p cal_accel_bias_x:=-12.11120 \
  -p cal_accel_bias_y:=3.55178 \
  -p cal_accel_bias_z:=-2.27557
```

### Camera node
```bash
ros2 run ugv1_controller capture_node --ros-args -p publish_raw:=true
```

### OpenVINS
```bash
ros2 run ov_msckf run_subscribe_msckf --ros-args \
  -p config_path:=/home/ubuntu/ros2_ws/src/ugv1_controller/config/estimator_config.yaml
```

Hold robot still at startup, then give a brief gentle shake to trigger initialization.

## OpenVINS Setup

OpenVINS cannot be compiled on the Pi (1.8GB RAM) or Mac M1 Docker (insufficient memory). Compile on a Google Cloud T2A ARM64 VM (Ubuntu 24.04, 16GB RAM):

```bash
# On VM
colcon build --packages-select ov_core ov_init ov_msckf \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_OV_EVAL=OFF \
  -DDISABLE_MATPLOTLIB=ON \
  -DENABLE_DYNAMIC_INIT=OFF

# Copy to Pi via Mac
gcloud compute scp --recurse sd0630@openvins-build:/home/sd0630/ros2_ws/install/ ~/ros2_ws/openvins_install/ --zone=asia-southeast1-c
rsync -av ~/ros2_ws/openvins_install/ ubuntu@<pi-ip>:/home/ubuntu/ros2_ws/install/
```

OpenVINS master branch requires patching for ROS2 Jazzy compatibility:
- `image_transport/image_transport.h` → `.hpp` in ROS2Visualizer.h and ROS1Visualizer.h
- `tf2_geometry_msgs/tf2_geometry_msgs.h` → `.hpp` in ROS2Visualizer.h and ROSVisualizerHelper.h
- `tf/transform_broadcaster.h` → `tf2_ros/transform_broadcaster.h` in ROSVisualizerHelper.h

## Calibration

Camera-IMU calibration done with Kalibr. Results in `config/kalibr_imucam_chain.yaml`:
- Camera: pinhole-radtan, 640x480, 30fps
- Timeshift: -0.185s
- Reprojection error: 0.705px

## Notes

- Static initialization only (ENABLE_DYNAMIC_INIT=OFF) — robot must be completely stationary at startup
- Pi swap: 4GB (`/swapfile`)
- Pi temperature at full load: ~58°C without heatsink
