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
- `motor_node` — subscribes to `/cmd_vel`, sends UART packets to Arduino

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
