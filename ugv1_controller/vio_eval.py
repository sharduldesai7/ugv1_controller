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
import math

import numpy as np


def associate(t_est, t_gt, tol=0.02):
    """Indices (i_est, i_gt) of the nearest ground-truth sample for each estimate."""
    t_est, t_gt = np.asarray(t_est), np.asarray(t_gt)
    j = np.clip(np.searchsorted(t_gt, t_est), 1, len(t_gt) - 1)
    j -= (np.abs(t_gt[j - 1] - t_est) < np.abs(t_gt[j] - t_est))
    keep = np.abs(t_gt[j] - t_est) <= tol
    return np.nonzero(keep)[0], j[keep]


def align_yaw(est, gt, with_scale):
    """Fit gt ~ s * Rz(theta) * est + t. Returns (s, theta, t) with s = 1 unless with_scale."""
    e, g = np.asarray(est, float), np.asarray(gt, float)
    me, mg = e[:, :2].mean(0), g[:, :2].mean(0)
    ec, gc = e[:, :2] - me, g[:, :2] - mg
    # Optimal rotation about z from the 2D cross-covariance
    sxx = (ec[:, 0] * gc[:, 0] + ec[:, 1] * gc[:, 1]).sum()
    sxy = (ec[:, 0] * gc[:, 1] - ec[:, 1] * gc[:, 0]).sum()
    theta = math.atan2(sxy, sxx)
    c, s_ = math.cos(theta), math.sin(theta)
    r = np.array([[c, -s_], [s_, c]])
    s = 1.0
    if with_scale:
        s = float((gc * (ec @ r.T)).sum() / (ec ** 2).sum())
    t = np.zeros(3)
    t[:2] = mg - s * (r @ me)
    t[2] = g[:, 2].mean() - s * e[:, 2].mean()
    return s, theta, t


def apply(est, s, theta, t):
    c, s_ = math.cos(theta), math.sin(theta)
    r = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])
    return s * (np.asarray(est, float) @ r.T) + t


def path_length(p):
    return float(np.linalg.norm(np.diff(np.asarray(p), axis=0), axis=1).sum())


def score(t_est, p_est, t_gt, p_gt, tol=0.02):
    ie, ig = associate(t_est, t_gt, tol)
    if len(ie) < 10:
        raise ValueError(f'only {len(ie)} estimate samples match the ground truth in time')
    e, g = np.asarray(p_est)[ie], np.asarray(p_gt)[ig]
    out = {'samples': len(ie), 'est_total': len(t_est)}
    s0, th0, tr0 = align_yaw(e, g, False)
    a0 = apply(e, s0, th0, tr0)
    out['ate_rmse_m'] = float(np.sqrt(((a0 - g) ** 2).sum(1).mean()))
    s1, th1, tr1 = align_yaw(e, g, True)
    a1 = apply(e, s1, th1, tr1)
    out['scale'] = s1
    out['ate_rmse_scaled_m'] = float(np.sqrt(((a1 - g) ** 2).sum(1).mean()))
    out['heading_offset_deg'] = math.degrees(th0)
    out['est_path_m'] = path_length(e)
    out['gt_path_m'] = path_length(g)
    out['path_ratio'] = out['est_path_m'] / out['gt_path_m']
    # Closed loop: where the estimate ends relative to where it started
    out['end_gap_est_m'] = float(np.linalg.norm(e[-1] - e[0]))
    out['end_gap_gt_m'] = float(np.linalg.norm(g[-1] - g[0]))
    out['est_duration_s'] = float(t_est[ie[-1]] - t_est[ie[0]])
    return out


def latency(est_stamps, est_recv, imu_stamps, imu_recv):
    """Per estimate: arrival time minus arrival time of the IMU sample with the same stamp (s)."""
    est_stamps, imu_stamps = np.asarray(est_stamps), np.asarray(imu_stamps)
    j = np.clip(np.searchsorted(imu_stamps, est_stamps), 0, len(imu_stamps) - 1)
    return np.asarray(est_recv) - np.asarray(imu_recv)[j]


# ── Bag reading (needs ROS) ───────────────────────────────────────────────────
def read_bag(path, topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    # record_sim_bag.sh makes zstd-compressed bags; the OpenVINS output bag is plain.
    # The compression reader rejects plain bags, so pick by the bag's metadata.
    compressed = False
    try:
        compressed = bool(rosbag2_py.Info().read_metadata(path, '').compression_format)
    except Exception:
        pass
    if compressed and hasattr(rosbag2_py, 'SequentialCompressionReader'):
        reader = rosbag2_py.SequentialCompressionReader()
    else:
        reader = rosbag2_py.SequentialReader()
    storage = rosbag2_py.StorageOptions(uri=path, storage_id='')
    conv = rosbag2_py.ConverterOptions('', '')
    reader.open(storage, conv)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    out = {t: [] for t in topics}
    msg_types = {t: get_message(types[t]) for t in topics if t in types}
    while reader.has_next():
        topic, data, recv_ns = reader.read_next()
        out[topic].append((deserialize_message(data, msg_types[topic]), recv_ns * 1e-9))
    return out


def odom_arrays(items):
    t = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m, _ in items])
    p = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z] for m, _ in items])
    return t, p


def print_timeline(t_est, p_est, t_gt, p_gt, step, ori_est=None, ori_gt=None):
    """Where the estimate leaves the truth: speed and distance from start, side by side."""
    cols = f"\n{'t (s)':>7} {'est speed':>10} {'true speed':>11} {'est dist':>10} {'true dist':>10}"
    if ori_est is not None:
        cols += f" {'est turn':>9} {'true turn':>10} {'est tilt':>9} {'true tilt':>10}"
    print(cols + "   (m/s, m from start, deg turned since start, deg tilt from vertical)")
    t0 = t_est[0]
    for k in range(int((t_est[-1] - t0) // step) + 1):
        t = t0 + k * step
        ie = min(int(np.searchsorted(t_est, t)), len(t_est) - 1)
        ig = min(int(np.searchsorted(t_gt, t_est[ie])), len(t_gt) - 1)
        je = min(int(np.searchsorted(t_est, t_est[ie] + 1.0)), len(t_est) - 1)
        jg = min(int(np.searchsorted(t_gt, t_gt[ig] + 1.0)), len(t_gt) - 1)
        ve = np.linalg.norm(p_est[je] - p_est[ie]) / max(t_est[je] - t_est[ie], 1e-6)
        vg = np.linalg.norm(p_gt[jg] - p_gt[ig]) / max(t_gt[jg] - t_gt[ig], 1e-6)
        row = f"{t_est[ie] - t0:7.0f} {ve:10.2f} {vg:11.2f} {np.linalg.norm(p_est[ie] - p_est[0]):10.1f} {np.linalg.norm(p_gt[ig] - p_gt[0]):10.1f}"
        if ori_est is not None:
            (ye, te), (yg, tg) = ori_est, ori_gt
            row += f" {ye[ie] - ye[0]:9.1f} {yg[ig] - yg[0]:10.1f} {te[ie]:9.2f} {tg[ig]:10.2f}"
        print(row)


def odom_yaw_tilt(items):
    """Unwrapped heading (deg) and tilt of the body z axis from vertical (deg) per message."""
    q = np.array([[m.pose.pose.orientation.x, m.pose.pose.orientation.y,
                   m.pose.pose.orientation.z, m.pose.pose.orientation.w] for m, _ in items])
    x, y, z, w = q.T
    yaw = np.degrees(np.unwrap(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))))
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1, 1)))
    return yaw, tilt


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
