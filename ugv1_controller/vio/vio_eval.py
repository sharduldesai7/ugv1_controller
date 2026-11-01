#!/usr/bin/env python3
"""Score an OpenVINS run against the simulator's ground truth.

  ros2 run ugv1_controller vio_eval --truth <sim bag> --est <openvins output bag>

--truth is the bag recorded in the simulation (topic /ground_truth/odom).
--est is a bag recorded while OpenVINS ran on the replayed data (topic
/odomimu, plus /imu/data_raw if you want processing latency).

OpenVINS starts in its own gravity-aligned frame with an arbitrary heading and
origin, so the estimate is first aligned to the ground truth by a rotation about
z and a translation (and, for the second figure, a scale). Everything except the
bag reading is plain numpy and is covered by test/test_vio_eval.py.
"""

import argparse

import numpy as np

from ugv1_controller.eval_common import odom_arrays, odom_yaw_tilt, print_timeline, read_bag, score


def latency(est_stamps, est_recv, imu_stamps, imu_recv):
    """Per estimate: arrival time minus arrival time of the IMU sample with the same stamp (s)."""
    est_stamps, imu_stamps = np.asarray(est_stamps), np.asarray(imu_stamps)
    j = np.clip(np.searchsorted(imu_stamps, est_stamps), 0, len(imu_stamps) - 1)
    return np.asarray(est_recv) - np.asarray(imu_recv)[j]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--truth', required=True)
    ap.add_argument('--est', required=True)
    ap.add_argument('--truth-topic', default='/ground_truth/odom')
    ap.add_argument('--est-topic', default='/odomimu')
    ap.add_argument('--timeline', type=float, default=0.0, metavar='SEC',
                    help='also print estimated and true speed and distance from start every SEC seconds')
    args = ap.parse_args()

    gt = read_bag(args.truth, [args.truth_topic])[args.truth_topic]
    est_bag = read_bag(args.est, [args.est_topic, '/imu/data_raw'])
    est = est_bag[args.est_topic]
    if not gt or not est:
        raise SystemExit(f'no messages: truth {len(gt)}, estimate {len(est)}')
    t_gt, p_gt = odom_arrays(gt)
    t_est, p_est = odom_arrays(est)
    r = score(t_est, p_est, t_gt, p_gt)

    print(f"estimates matched      {r['samples']} of {r['est_total']}")
    print(f"duration               {r['est_duration_s']:.0f} s")
    print(f"path length            truth {r['gt_path_m']:.2f} m, estimate {r['est_path_m']:.2f} m (ratio {r['path_ratio']:.2f})")
    print(f"scale                  {r['scale']:.3f}   (1.000 = correct; the real rover read about 2)")
    print(f"ATE, no scale fit      {r['ate_rmse_m']:.3f} m RMSE")
    print(f"ATE, scale fitted      {r['ate_rmse_scaled_m']:.3f} m RMSE")
    print(f"closed-loop gap        estimate {r['end_gap_est_m']:.2f} m, truth {r['end_gap_gt_m']:.2f} m")
    print(f"heading offset         {r['heading_offset_deg']:.1f} deg (OpenVINS frame vs world)")
    if args.timeline > 0:
        print_timeline(t_est, p_est, t_gt, p_gt, args.timeline, odom_yaw_tilt(est), odom_yaw_tilt(gt))
    imu = est_bag['/imu/data_raw']
    if imu:
        ti = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m, _ in imu])
        ri = np.array([rt for _, rt in imu])
        lat = latency(t_est, [rt for _, rt in est], ti, ri)
        print(f"processing latency     median {np.median(lat) * 1e3:.0f} ms, 95th {np.percentile(lat, 95) * 1e3:.0f} ms, max {lat.max() * 1e3:.0f} ms")
        span = t_est[-1] - t_est[0]
        print(f"estimate rate          {len(t_est) / span:.1f} Hz over {span:.0f} s of data")


if __name__ == '__main__':
    main()
