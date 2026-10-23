#!/usr/bin/env python3
"""Mean and spread of the IMU while the rover is at rest, from a recorded sim bag.

Usage: python3 scripts/sim/imu_rest_stats.py ~/sim_bags/aisles1 [seconds=15]

Looks at the first and the last SECONDS of /imu/data_raw (the rover is stationary then).
At rest the IMU frame should read accel = (0, 0, 9.81) and gyro = (0, 0, 0). A mean that
is off by 0.1 m/s^2 or more is a bias the filter has to estimate; the spread should match
the noise settings (accel about 7.5e-3, gyro about 1.1e-3 per sample at 100 Hz).
"""
import sys
import numpy as np
from ugv1_controller.vio_eval import read_bag


def stats(a, g, label):
    ma, mg = a.mean(axis=0), g.mean(axis=0)
    print(f'{label}: {len(a)} samples')
    print(f'  accel mean  x {ma[0]:+.4f}  y {ma[1]:+.4f}  z {ma[2]:+.4f}   |a| {np.linalg.norm(ma):.4f}  '
          f'(error vs 9.81: {np.linalg.norm(ma) - 9.81:+.4f})')
    print(f'  accel std   x {a[:, 0].std():.4f}  y {a[:, 1].std():.4f}  z {a[:, 2].std():.4f}')
    print(f'  gyro mean   x {mg[0]:+.5f}  y {mg[1]:+.5f}  z {mg[2]:+.5f}')
    print(f'  gyro std    x {g[:, 0].std():.5f}  y {g[:, 1].std():.5f}  z {g[:, 2].std():.5f}')


def main():
    path = sys.argv[1]
    sec = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0
    msgs = read_bag(path, ['/imu/data_raw'])['/imu/data_raw']
    t = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m, _ in msgs])
    a = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for m, _ in msgs])
    g = np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for m, _ in msgs])
    t -= t[0]
    first = t < sec
    last = t > t[-1] - sec
    stats(a[first], g[first], f'first {sec:.0f} s')
    stats(a[last], g[last], f'last {sec:.0f} s')


if __name__ == '__main__':
    main()
