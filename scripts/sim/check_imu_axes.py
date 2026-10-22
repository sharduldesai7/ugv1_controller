#!/usr/bin/env python3
"""Simulation check: IMU axis directions and the 180 degree IMU mounting.

Needs the simulation running (sim.launch.py). Drives the rover forward for about 1.5 s,
then turns it left in place for about 1.5 s, and checks:

  rest:      accel z about +9.81 (gravity along the IMU's up axis)
  forward:   accel x goes NEGATIVE while speeding up (IMU x points to the back of the rover)
  turn left: gyro z goes POSITIVE

Run it, then restart the simulation before recording a route: the rover has moved.

  python3 scripts/sim/check_imu_axes.py
"""

import statistics
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)


class Check(Node):
    def __init__(self):
        super().__init__('check_imu_axes')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Imu, '/imu/data_raw', self.on_imu, QOS)
        self.samples = []

    def on_imu(self, m):
        a, g = m.linear_acceleration, m.angular_velocity
        self.samples.append((a.x, a.y, a.z, g.z))

    def phase(self, seconds, lin, ang):
        self.samples = []
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            t = Twist()
            t.linear.x, t.angular.z = lin, ang
            self.pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.05)
        return list(self.samples)


def report(name, ok, detail):
    print(f'{"PASS" if ok else "FAIL"}  {name}: {detail}')
    return ok


def main():
    rclpy.init()
    n = Check()
    ok = True
    rest = n.phase(2.0, 0.0, 0.0)
    if len(rest) < 20:
        print('FAIL  no IMU data; is the simulation running?')
        return
    ax0 = statistics.mean(s[0] for s in rest)
    ay0 = statistics.mean(s[1] for s in rest)
    az0 = statistics.mean(s[2] for s in rest)
    ok &= report('rest: gravity on +z', 9.5 < az0 < 10.1, f'mean accel x {ax0:+.3f} y {ay0:+.3f} z {az0:+.3f}')

    fwd = n.phase(1.5, 0.3, 0.0)
    dax = min(s[0] for s in fwd) - ax0
    ok &= report('forward: accel x negative while speeding up', dax < -0.4,
                 f'change in accel x {dax:+.2f} m/s^2 (expect about -1.0)')

    n.phase(2.0, 0.0, 0.0)
    turn = n.phase(1.5, 0.0, 0.5)
    gz = statistics.mean(s[3] for s in turn[len(turn) // 3:])
    ok &= report('turn left: gyro z positive', 0.25 < gz < 0.8, f'mean gyro z {gz:+.2f} rad/s (expect about +0.5)')

    n.phase(0.5, 0.0, 0.0)
    print('\nAll checks passed.' if ok else '\nSomething is wrong with the IMU frame; send me this output.')
    print('Restart the simulation before recording: the rover has moved.')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
