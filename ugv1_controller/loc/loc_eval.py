#!/usr/bin/env python3
"""Score the standard localisation stack against the simulator's ground truth.

  ros2 run ugv1_controller loc_eval --truth <sim bag> --est <run bag> [--odom-topic /localization/odom_pose]

--truth is the bag recorded in the simulation (/ground_truth/odom).
--est is the bag recorded while the stack ran (/localization/map_pose, and
/localization/odom_pose for the odometry-only comparison).

slam_toolbox's map frame starts at the rover's first pose, so the estimate is
aligned to the truth by one rotation about z and one translation (no scale). The
error is then the planar distance between the aligned estimate and the truth.
"""

import argparse

import numpy as np

from ugv1_controller.eval_common import align_yaw, apply, associate, odom_arrays, read_bag


def error_timeline(t_est, p_est, t_gt, p_gt, tol=0.05):
    """Planar error (m) of the aligned estimate, with time and distance travelled by the truth."""
    ie, ig = associate(t_est, t_gt, tol)
    if len(ie) < 10:
        raise ValueError(f'only {len(ie)} estimate samples match the ground truth in time')
    e, g = np.asarray(p_est, float)[ie], np.asarray(p_gt, float)[ig]
    s, th, tr = align_yaw(e, g, False)
    a = apply(e, s, th, tr)
    err = np.linalg.norm(a[:, :2] - g[:, :2], axis=1)
    step = np.linalg.norm(np.diff(np.asarray(p_gt, float)[:, :2], axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(step)])
    return {'t': np.asarray(t_est)[ie], 'err': err, 'dist': cum[ig], 'heading_offset': th}


def error_stats(tl):
    err, dist = tl['err'], tl['dist']
    travelled = float(dist[-1] - dist[0])
    return {
        'samples': int(len(err)),
        'rmse_m': float(np.sqrt((err ** 2).mean())),
        'median_m': float(np.median(err)),
        'p95_m': float(np.percentile(err, 95)),
        'max_m': float(err.max()),
        'final_m': float(err[-1]),
        'distance_m': travelled,
        'final_pct_of_distance': float(100.0 * err[-1] / travelled) if travelled > 0 else float('nan'),
    }


def _report(label, stats):
    print(f"{label}")
    print(f"  samples {stats['samples']}, truth distance {stats['distance_m']:.1f} m")
    print(f"  error   RMSE {stats['rmse_m']:.3f} m, median {stats['median_m']:.3f} m, "
          f"95th {stats['p95_m']:.3f} m, max {stats['max_m']:.3f} m")
    print(f"  final   {stats['final_m']:.3f} m ({stats['final_pct_of_distance']:.2f}% of distance)")


def _print_timeline(tl, step):
    print(f"\n{'t (s)':>7} {'dist (m)':>9} {'error (m)':>10}")
    t0 = tl['t'][0]
    for k in range(int((tl['t'][-1] - t0) // step) + 1):
        i = min(int(np.searchsorted(tl['t'], t0 + k * step)), len(tl['t']) - 1)
        print(f"{tl['t'][i] - t0:7.0f} {tl['dist'][i]:9.1f} {tl['err'][i]:10.3f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--truth', required=True)
    ap.add_argument('--est', required=True)
    ap.add_argument('--truth-topic', default='/ground_truth/odom')
    ap.add_argument('--est-topic', default='/localization/map_pose')
    ap.add_argument('--odom-topic', default='/localization/odom_pose',
                    help='odometry-only pose to compare against (empty string to skip)')
    ap.add_argument('--timeline', type=float, default=0.0, metavar='SEC',
                    help='also print the error every SEC seconds')
    args = ap.parse_args()

    topics = [args.truth_topic]
    gt = read_bag(args.truth, topics)[args.truth_topic]
    est_topics = [args.est_topic] + ([args.odom_topic] if args.odom_topic else [])
    est_bag = read_bag(args.est, est_topics)
    if not gt or not est_bag[args.est_topic]:
        raise SystemExit(f'no messages: truth {len(gt)}, estimate {len(est_bag[args.est_topic])}')
    t_gt, p_gt = odom_arrays(gt)

    tl = error_timeline(*odom_arrays(est_bag[args.est_topic]), t_gt, p_gt)
    _report(f'map -> base_link ({args.est_topic})', error_stats(tl))
    print(f"  heading offset of map frame vs world {np.degrees(tl['heading_offset']):.1f} deg")
    if args.odom_topic and est_bag[args.odom_topic]:
        tlo = error_timeline(*odom_arrays(est_bag[args.odom_topic]), t_gt, p_gt)
        _report(f'odom -> base_link ({args.odom_topic}), no map correction', error_stats(tlo))
    if args.timeline > 0:
        _print_timeline(tl, args.timeline)


if __name__ == '__main__':
    main()
