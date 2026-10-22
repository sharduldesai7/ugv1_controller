# UGV1 simulation

A Gazebo Harmonic model of the rover in a textured warehouse, used to test the
software stack (OpenVINS, later RTAB-Map and Nav2) without the real chassis.

**What it proves and what it does not.** The simulated IMU has the noise levels
from your Kalibr calibration, but none of the real unknowns: vibration from
worn motors, wheel slip, rolling shutter, motion blur, exposure changes, a
camera-IMU time offset that drifts. A good score here means the software chain
(frames, topics, extrinsics, Pi performance, odometry TF) is right. It does not
predict the accuracy on the real rover. A bad score here is a real bug.

## How it fits together

```
 Simulation machine (UTM VM)                                Raspberry Pi 4
 ---------------------------                                --------------
 gz sim (headless) --bridge--> /imu/data_raw                 ros2 bag play (1x)
   rover + warehouse           /video_raw_raw   --record-->  --> OpenVINS --> /odomimu
   scripted route              /scan                              odom_tf_node --> odom -> base_link
   /cmd_vel <- route node      /ground_truth/odom                 CPU logger
              or web /drive                                  ros2 bag record /odomimu ...
                                                                  |
                            vio_eval (truth bag vs estimate bag) <-+
```

Simulated time stamps the bag, so the simulation may run slower than real time;
the Pi then replays it at exactly 1x, which is the load the real rover would
put on it.

## What is simulated

| Part | Setting | Source |
| --- | --- | --- |
| Frames | `base_link` at the IMU; IMU frame = base_link turned 180 deg about z; `laser` as in `lidar.launch.py` | real rover |
| Camera | 640x480, 15 fps, mono8, ideal pinhole, f = 396.6 px, no distortion, no time offset, small pixel noise | focal length from your calibration |
| Camera pose | at the real `T_imu_cam` (about 7.9 deg nose-up, 6.7 cm forward of the IMU) | `config/kalibr_imucam_chain.yaml` |
| IMU | 100 Hz, white noise 1.134e-4 / 7.5e-4 per sqrt(Hz), bias random walk 3.0e-5 / 8.0e-4 | `config/kalibr_imu_chain.yaml` |
| LiDAR | 720 samples, 7.4 Hz, 0.15 to 12 m, 5 mm noise | A1M8 |
| Drive | two-wheel differential drive with casters, 65 mm wheels, 0.22 m track, limited acceleration | assumed; the real chassis is being replaced |
| World | 14 x 10 m room, three racks of boxes with 1.8 m aisles, textured floor, walls and boxes | generated |

Not matched on purpose: the real principal point (the real cy = 269 is 29 px
off centre) and lens distortion. The simulated camera is an ideal pinhole, and
`config/sim/kalibr_imucam_chain.yaml` says so. Real distortion can be added
later once the base case works.

## One-time setup on the simulation machine

Ubuntu 24.04 with ROS 2 Jazzy (you already have this in the UTM VM):

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-tf2-ros python3-opencv python3-flask xvfb
cd ~/ros2_ws/src/ugv1_controller && git pull
cd ~/ros2_ws && colcon build --packages-select ugv1_controller --symlink-install && source install/setup.bash
```

If you have not pulled the OpenVINS workspace there, nothing else is needed:
the simulation machine does not run OpenVINS in this plan.

## If Gazebo crashes at start-up (rendering)

The cameras and LiDAR are rendered, so Gazebo needs an OpenGL context even with
no window. In a VM without a GPU the default can segfault inside `libEGL_mesa`.
Try, in this order, until `gz` stays up:

```bash
ros2 launch ugv1_controller sim.launch.py                  # render:=egl (default)
ros2 launch ugv1_controller sim.launch.py render:=xvfb     # virtual display, software OpenGL
ros2 launch ugv1_controller sim.launch.py gui:=true        # with a desktop session in the VM
```

In a UTM VM the usual cause is Mesa choosing `virgl` (OpenGL 2.1 only), which shows
up as `OpenGL 3.3 is not supported` or a segfault in `libEGL_mesa`. `render:=xvfb`
forces the software renderer (llvmpipe). If that still crashes, shut the VM down and
in UTM change Devices > Display > Emulated Display Card to `virtio-ramfb` (no GPU
acceleration), so only llvmpipe exists; the desktop is a little slower, Gazebo works.

The `gz_frame_id ... not defined in SDF` lines are warnings and can be ignored;
check 2 below shows whether the frame names came out right.

## Alternative: run the simulation in a Docker container on the Mac

If the UTM VM's graphics will not work, `sim/docker/` builds a native arm64 Ubuntu 24.04
container with ROS 2 Jazzy, Gazebo Harmonic and a VNC desktop. It has no GPU device, so
rendering is software and works. From a clone of this repository on the Mac:

```bash
sim/docker/run.sh                      # builds the image, starts the container 'ugv1-sim'
docker exec -it ugv1-sim bash          # then follow the printed steps (colcon build, launch)
```

Bags are written to `~/sim_bags` on the Mac; copy them to the Pi with `scp -r`. The
record script expects `~/ros2_ws` inside the container, which is where the image puts it.

## Run it

```bash
# terminal 1: simulator, bridge, web page; the rover drives the scripted route
ros2 launch ugv1_controller sim.launch.py route:=true

# terminal 2: record (Ctrl-C when the route node says "Route finished", about 4 min of simulated time)
~/ros2_ws/src/ugv1_controller/scripts/sim/record_sim_bag.sh aisles1
```

To drive by hand instead, use `sim.launch.py` without `route:=true` and open
`http://<vm-ip>:8080/drive`; the same page, keys and Drive Enable rules apply.
With `gui:=true` the Gazebo window opens (slow without a GPU).

Copy the bag to the Pi and replay:

```bash
scp -r ~/sim_bags/aisles1 ubuntu@<pi-ip>:~/sim_bags/          # from the VM or Mac
# on the Pi (after git pull and colcon build; stop the sensor nodes first):
~/ros2_ws/src/ugv1_controller/scripts/sim/replay_on_pi.sh ~/sim_bags/aisles1 run1
ros2 run ugv1_controller vio_eval --truth ~/sim_bags/aisles1 --est ~/sim_runs/run1/openvins_out
```

`vio_eval` prints scale, position error against ground truth, the closed-loop
gap, processing latency on the Pi and the OpenVINS output rate. The Pi's CPU
and memory use are in `~/sim_runs/run1/cpu.csv`, summarised when the script ends.

## Check the simulation itself first

Do these before trusting any score. Each takes a minute.

1. **Sensors alive** (VM): `ros2 topic hz /imu/data_raw` (about 100 at real-time factor 1; lower if the simulation is slow), `/video_raw_raw` (15), `/scan` (7.4).
2. **Frame names:** `ros2 topic echo --once /imu/data_raw --field header.frame_id` should say `imu_link`; `/scan` should say `laser`; `/video_raw_raw` `camera_frame`. If they show a long `ugv1_rover/base_link/...` name, the `gz_frame_id` tag is not honoured by your Gazebo version; tell me and I will add a relay.
3. **Gravity and axes, rover at rest:** `ros2 topic echo --once /imu/data_raw --field linear_acceleration` should be about `x 0, y 0, z +9.81`. If z is -9.81 or x/y carry gravity, the IMU frame is wrong.
4. **IMU turned 180 deg:** publish `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"` and watch `linear_acceleration.x` while it speeds up: it must go **negative** (rover accelerates toward base +x, IMU x points backward). Gyro z must go positive when you send `angular: {z: 0.5}` (turning left).
5. **LiDAR orientation:** run `python3 scripts/lidar_nearest.py` (it works on the simulated `/scan` too). At the start the nearest thing is the wall about 0.9 m behind the rover, so the robot-frame angle must read about 180 deg.
6. **See what the camera sees:** open `http://<vm-ip>:8080/drive`. Floor, boxes and walls should show texture; if it is mostly flat grey, lighting or texture loading failed (check for `Unable to find file` in the launch output).
7. **Ground truth:** `ros2 topic echo --once /ground_truth/odom --field pose.pose.position` should be near `x -6, y -3.8, z 0.1` at the start. Note that Gazebo may report the pose relative to the world or the start; the score only needs it to be consistent.

## Regenerating

`python3 sim/tools/make_sim_assets.py` rewrites the models, world, textures and
`config/sim/` from the real calibration. Options: `--imu-noise-scale 0` for a
noiseless IMU (isolates frame and extrinsic bugs from noise), `--camera-format R8G8B8`
if the bridge rejects mono8.

## Known limits and things to verify

- Written without being able to run Gazebo or ROS; first run may need small fixes. Send me the first error text.
- Software rendering may be slow (a fraction of real time). That only lengthens recording, not the replay.
- Ground-truth publisher and `gz_frame_id` behaviour are the two Gazebo details most likely to differ from what I expect (checks 2 and 7).
- OpenVINS needs the rover still for the first 5 s; the route node waits `start_delay` (5 s) of simulated time.
