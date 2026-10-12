#!/usr/bin/env python3
"""
Print the nearest LiDAR point once a second, in both the LiDAR's own frame and
the robot frame (base_link), to check the mounting orientation.

Robot frame: 0 deg = straight ahead, +90 deg = left, 180 deg = behind
(counter-clockwise positive, ROS convention).

The robot-frame angle uses the base_link -> laser transform published by
launch/lidar.launch.py, so the driver and that launch file must be running:

  ros2 launch ugv1_controller lidar.launch.py
  python3 scripts/lidar_nearest.py
"""

import math

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan


def wrap_deg(angle):
    """Wrap an angle in degrees to the range (-180, 180]."""
    return 180.0 - ((180.0 - angle) % 360.0)


def to_robot_angle_deg(laser_angle_deg, yaw_rad):
    """Convert an angle in the laser frame to the robot frame.

    yaw_rad is the rotation of the laser frame about z relative to base_link.
    """
    return wrap_deg(laser_angle_deg + math.degrees(yaw_rad))


class Nearest(Node):

    def __init__(self):
        super().__init__('lidar_nearest')
        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer, self)
        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self._last = 0.0

    def _on_scan(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last < 1.0:
            return
        self._last = now

        best = None
        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max and (best is None or r < best[1]):
                best = (i, r)
        if best is None:
            print('no valid points in the scan')
            return
        laser_deg = math.degrees(msg.angle_min + best[0] * msg.angle_increment)

        try:
            tf = self._buffer.lookup_transform(
                'base_link', msg.header.frame_id, Time())
        except Exception as exc:  # transform not available yet
            print('nearest: %.2f m at %.0f deg (laser frame); '
                  'no base_link transform yet: %s' % (best[1], laser_deg, exc))
            return
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        print('nearest: %.2f m | laser frame %4.0f deg | robot frame %4.0f deg'
              % (best[1], laser_deg, to_robot_angle_deg(laser_deg, yaw)))


def main():
    rclpy.init()
    node = Nearest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
