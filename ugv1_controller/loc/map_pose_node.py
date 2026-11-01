#!/usr/bin/env python3
"""Publish the localisation result as Odometry messages so it can be recorded and scored.

  /localization/map_pose   map  -> base_link   (the full result: EKF plus slam_toolbox correction)
  /localization/odom_pose  odom -> base_link   (odometry only, for comparing drift)

Each pose is published once per new transform time stamp, taken from the transform itself,
so it lines up with the ground truth on simulated time.
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformException


class MapPoseNode(Node):
    def __init__(self):
        super().__init__('map_pose_node')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('rate_hz', 20.0)
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.map_pub = self.create_publisher(Odometry, '/localization/map_pose', 50)
        self.odom_pub = self.create_publisher(Odometry, '/localization/odom_pose', 50)
        self.last = {}
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.tick)

    def _publish(self, pub, parent):
        try:
            t = self.buffer.lookup_transform(parent, self.base_frame, Time())   # latest available
        except TransformException:
            return
        stamp_ns = t.header.stamp.sec * 1_000_000_000 + t.header.stamp.nanosec
        if self.last.get(parent) == stamp_ns:
            return
        self.last[parent] = stamp_ns
        m = Odometry()
        m.header.stamp = t.header.stamp
        m.header.frame_id = parent
        m.child_frame_id = self.base_frame
        m.pose.pose.position.x = t.transform.translation.x
        m.pose.pose.position.y = t.transform.translation.y
        m.pose.pose.position.z = t.transform.translation.z
        m.pose.pose.orientation = t.transform.rotation
        pub.publish(m)

    def tick(self):
        self._publish(self.map_pub, self.map_frame)
        self._publish(self.odom_pub, self.odom_frame)


def main():
    rclpy.init()
    node = MapPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
