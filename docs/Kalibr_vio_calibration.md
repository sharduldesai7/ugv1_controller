# Kalibr Calibration Runbook — UGV1

Complete step-by-step guide to calibrate camera intrinsics and camera-IMU extrinsics for UGV1. Run this whenever the IMU is replaced, the camera is repositioned, or the mount changes physically.

---

## Prerequisites

### Hardware required
- UGV1 with camera and IMU mounted in their final positions (do not move them after calibration)
- AprilGrid calibration target — print `kalibr_target_april_6x6_A4.pdf` at exactly 100% scale on A4 paper, mount flat on a rigid board (cardboard or foam board). Do not laminate — reflections kill calibration.
- Tape measure or calipers
- Good consistent lighting — no flickering, no direct sun, no blown highlights

### Measure the AprilGrid physically
Before running anything, measure the printed target with calipers:
- **Tag size** — measure one black square side in metres (e.g. `0.022m`)
- **Tag spacing** — measure the white gap between two adjacent tags in metres (e.g. `0.003m`)
- **Grid layout** — count rows and columns of tags (e.g. 6×6)

Record these — they go into `april_6x6.yaml`.

### Software required (Mac)
- Docker Desktop running
- Kalibr Docker image: `docker pull stereolabs/kalibr`
- `rosbags` Python package: `pip3 install rosbags`

---

## Step 1: Prepare the AprilGrid config file

Create `~/kalibr_ws/april_6x6.yaml`:

```yaml
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.022        # replace with your measured value in metres
tagSpacing: 0.136     # ratio: spacing/tagSize (e.g. 0.003/0.022 = 0.136)
codeOffset: 0
```

---

## Step 2: Prepare the IMU config file

Create `~/kalibr_ws/imu.yaml` with your IMU's noise values.

For **MPU9250** (current IMU):
```yaml
rostopic: /imu/data_raw
update_rate: 100.0
accelerometer_noise_density: 2.0000e-3
accelerometer_random_walk: 3.0000e-3
gyroscope_noise_density: 1.6968e-04
gyroscope_random_walk: 1.9393e-05
```

For **BMI088** (if upgraded):
```yaml
rostopic: /imu/data_raw
update_rate: 100.0
accelerometer_noise_density: 1.4000e-3
accelerometer_random_walk: 1.2000e-3
gyroscope_noise_density: 8.0000e-05
gyroscope_random_walk: 1.0000e-5
```

For **ICM-42688-P** (if upgraded):
```yaml
rostopic: /imu/data_raw
update_rate: 100.0
accelerometer_noise_density: 7.0000e-4
accelerometer_random_walk: 8.0000e-4
gyroscope_noise_density: 6.0000e-06
gyroscope_random_walk: 1.0000e-7
```

---

## Step 3: Start nodes on the Pi

**Terminal 1 — IMU node (with calibration biases):**
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
Note: if replacing IMU, run with all biases at 0.0 and recalibrate first via `ros2 service call /imu/calibrate_accel std_srvs/srv/Trigger`.

**Terminal 2 — Camera node (compressed only, no raw needed for Kalibr):**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run ugv1_controller capture_node
```

Verify both topics are publishing:
```bash
ros2 topic hz /imu/data_raw
ros2 topic hz /video_raw
```
IMU should be ~100Hz, camera ~15-30Hz.

---

## Step 4: Record the calibration bag

**Terminal 3 — Record bag on Pi:**
```bash
ros2 bag record /imu/data_raw /video_raw -o ~/kalibr_cal_bag
```

### Camera-only calibration movement (first 2 minutes):
Hold the robot stationary and move the AprilGrid slowly in front of the camera. Cover all areas of the frame — corners, edges, centre. Tilt and rotate the target. Move slowly — no blur.

### Camera-IMU calibration movement (next 3 minutes):
Hold the AprilGrid stationary in view. Move the entire robot (camera + IMU together) with deliberate, varied motion:
- Slow rotations around X, Y, Z axes independently
- Slow translations forward/back, left/right, up/down
- Figure-8 patterns
- Avoid fast jerky motion — smooth and deliberate
- Keep the AprilGrid fully visible throughout

Stop recording:
```
Ctrl+C
```

---

## Step 5: Convert bag to ROS1 format

Kalibr requires ROS1 bag format. On the Mac:

**Copy bag from Pi:**
```bash
rsync -av ubuntu@192.168.29.4:~/kalibr_cal_bag/ ~/kalibr_ws/kalibr_cal_bag/
```

**Convert to ROS1:**
```bash
pip3 install rosbags
rosbags-convert ~/kalibr_ws/kalibr_cal_bag --dst ~/kalibr_ws/kalibr_cal_bag_ros1.bag
```

---

## Step 6: Run camera intrinsics calibration

```bash
docker run -it --rm \
  -v ~/kalibr_ws:/data \
  stereolabs/kalibr \
  kalibr_calibrate_cameras \
  --bag /data/kalibr_cal_bag_ros1.bag \
  --target /data/april_6x6.yaml \
  --models pinhole-radtan \
  --topics /video_raw \
  --show-extraction
```

### Accept/reject criteria:
- Reprojection error < 0.3 pixels — excellent
- Reprojection error 0.3–0.7 pixels — acceptable
- Reprojection error > 0.7 pixels — re-record, check target flatness and lighting

Current UGV1 result: **0.705px** (acceptable but borderline — aim for better next time).

Output file: `camchain-imucam-datakalibr_cal_bag_ros1.yaml`

---

## Step 7: Run camera-IMU extrinsics calibration

```bash
docker run -it --rm \
  -v ~/kalibr_ws:/data \
  stereolabs/kalibr \
  kalibr_calibrate_imu_camera \
  --bag /data/kalibr_cal_bag_ros1.bag \
  --cam /data/camchain-imucam-datakalibr_cal_bag_ros1.yaml \
  --imu /data/imu.yaml \
  --target /data/april_6x6.yaml
```

This produces the full `T_cam_imu` transform and time offset.

### Accept/reject criteria:
- Time offset should be stable and consistent with previous runs (UGV1 baseline: `-0.185s`)
- Reprojection error should not increase significantly from Step 6
- If time offset changes by >20ms from baseline, something is wrong — check IMU timestamp source

---

## Step 8: Update OpenVINS config files

Copy outputs into UGV1 config:

**`~/ros2_ws/src/ugv1_controller/config/kalibr_imucam_chain.yaml`:**
```yaml
%YAML:1.0
cam0:
  T_imu_cam:        # copy T_cam_imu matrix from Kalibr output
    - [r00, r01, r02, t0]
    - [r10, r11, r12, t1]
    - [r20, r21, r22, t2]
    - [0.0, 0.0, 0.0, 1.0]
  cam_overlaps: []
  camera_model: pinhole
  distortion_coeffs: [k1, k2, p1, p2]   # from Kalibr output
  distortion_model: radtan
  intrinsics: [fx, fy, cx, cy]           # from Kalibr output
  resolution: [640, 480]
  rostopic: /video_raw_raw
  timeshift_cam_imu: -0.185              # update with new value
```

**`~/ros2_ws/src/ugv1_controller/config/kalibr_imu_chain.yaml`:**
Update `update_rate` if IMU rate changed.

**Commit and push:**
```bash
cd ~/ros2_ws/src/ugv1_controller
git add config/
git commit -m "Update Kalibr calibration outputs — [date] [IMU model]"
git push
```

---

## Step 9: Verify on Pi

Restart OpenVINS and check:
1. Intrinsics printed at startup match new Kalibr values
2. Timeoffset matches new value
3. Stationary drift < 2cm/minute with ZUPT enabled

---

## Common Failures and Fixes

| Symptom | Likely cause | Fix |
|---|---|---|
| Reprojection error > 1.0px | Target not flat / moving during capture | Remount target on rigid board, move slower |
| Calibration fails to converge | Not enough motion variety | Add more axis rotations, especially roll |
| Time offset very different from baseline | IMU clock drift or wrong topic | Check `/imu/data_raw` timestamps are ROS clock, not system clock |
| OpenVINS drifts more after new calibration | Bad extrinsics | Check physical IMU-camera mount didn't shift between calibration and deployment |
| `T_cam_imu` Z translation > 0.1m | IMU and camera very far apart | Verify physical measurements, check coordinate frame conventions |

---

## Physical Measurement Checklist

Before each calibration session, verify nothing has shifted:
- [ ] Camera screw tight, no wobble
- [ ] IMU mount secure, no flex
- [ ] AprilGrid flat (no curl at edges)
- [ ] AprilGrid measured with calipers (not ruler)
- [ ] Lighting consistent, no flickering
- [ ] Pi clock synced (`timedatectl status` shows NTP synchronized)
