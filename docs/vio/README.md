# OpenVINS (monocular VIO): parked

**Status.** On 2026-10-25 the decision was to stop using monocular VIO for localisation and
move to a standard stack (LiDAR + IMU + wheel encoders, with the camera as a localisation aid).
The OpenVINS code, configuration and tools are kept here, separate from the rest of the package,
so the work can be picked up again.

## Where things are

| What | Where |
| --- | --- |
| `odom_tf_node` (OpenVINS `/odomimu` to `odom -> base_link`) | `ugv1_controller/vio/odom_tf_node.py` |
| `vio_eval` (score an OpenVINS run against simulator ground truth) | `ugv1_controller/vio/vio_eval.py` |
| Trajectory alignment and scoring shared with other evaluators | `ugv1_controller/eval_common.py` |
| Real rover estimator and Kalibr chain files | `config/vio/` |
| Simulation copies (ideal pinhole camera, same tuning) | `config/vio/sim/`, written by `sim/tools/make_sim_assets.py` |
| Old, unused configs (pending removal) | `config/vio/unused/` |
| Replay a sim bag into OpenVINS on the Pi | `scripts/vio/replay_on_pi.sh` |
| Contact sheet of camera frames with corner counts | `scripts/vio/dump_frames.py` |
| IMU mean and spread at rest from a bag | `scripts/vio/imu_rest_stats.py` |
| Kalibr calibration write-up | `docs/vio/Kalibr_vio_calibration.md` |

The command names are unchanged (`ros2 run ugv1_controller vio_eval`, `odom_tf_node`), but their
Python module paths moved, so run `colcon build --packages-select ugv1_controller --symlink-install`
once after pulling.

## Running it

Real rover: `scripts/start_ugv1.sh --with-openvins`.

Simulation, after recording a bag with `scripts/sim/record_sim_bag.sh`:

```bash
scripts/vio/replay_on_pi.sh ~/sim_bags/<bag> <run_name>
OVERRIDES="max_clones=12 try_zupt=false" scripts/vio/replay_on_pi.sh ~/sim_bags/<bag> <run_name>   # config experiments
ros2 run ugv1_controller vio_eval --truth ~/sim_bags/<bag> --est ~/sim_runs/<run_name>/openvins_out --timeline 5
```

`OVERRIDES` changes top-level keys of a copy of the sim estimator config in the run folder; the
repository is untouched. OpenVINS console output is kept in `~/sim_runs/<run_name>/openvins.log`.

## What the simulation showed (October 2026)

Replay at 1x on the Pi 4 against ground truth, 62.7 m closed route at 0.3 m/s.

| Run | Bag and IMU | `max_clones` | Result |
| --- | --- | --- | --- |
| run3 | `aisles1`, Gazebo IMU bias at full scale | 6 | Diverged from the start. Estimated speed 0.95 m/s at 10 s and over 100 m/s by 190 s; estimated path 22,262 m against 62.6 m; closed-loop gap 8.8 km. |
| run4 | `aisles_ideal`, noise-free and bias-free IMU | 6 | Accurate for about 25 s (0.28 to 0.31 m/s), speed then creeps up and runs away after the first corner (5.2 m/s at 50 s); `nan` from about 245 s. |
| run6 | `aisles_ideal` | 12 | No runaway until about 180 s, but speed 15 to 50% too high on the first leg and a stall (0.01 to 0.08 m/s while truth is 0.30) from 70 to 95 s; `nan` at about 185 s. |

Pi cost, mean over the run: about 50% of one core (400% is all four), peak 78 to 148%, about 110 MB.

Heading stayed within about 3 degrees of truth through every turn, and tilt error stayed under 2 degrees,
so the failures are in position and velocity, not orientation.

Ruled out or unlikely:
- IMU noise and bias: the noise-free IMU (run4, run6) diverged too. (The first bag did show the Gazebo
  bias wandering to about 6 mrad/s on the gyro and 0.1 m/s^2 on the accelerometer over 5 minutes, about
  10x the Kalibr random walk, so the generator default is now `--imu-bias-scale 0.1`.)
- Gravity setting (world and config both 9.81).
- Camera-IMU rotation and time offset: heading tracked correctly through the turns.
- ZUPT during the run4 failure window: no accepted ZUPT updates there.

Not resolved:
- The leading, unproven explanation is weak scale observability under constant-velocity straight-line
  motion, made worse by a long approach to a plain wall (one frame near the first corner had only 10 FAST
  corners on the raw image; the count after histogram equalisation, which OpenVINS uses, was not measured).
- A shorter feature window was a large factor in the run4 runaway (run6 with 12 clones did not run away).
- A replay with ZUPT off (`OVERRIDES="max_clones=12 try_zupt=false"`) was planned but never run.

This does not explain the real rover's earlier 2x scale error, since hand-carried tests had plenty of
acceleration. The camera-IMU time offset in the real calibration (-0.198 s) is the first thing to check there.
