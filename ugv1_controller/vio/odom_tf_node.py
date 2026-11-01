#!/usr/bin/env python3
"""
Turns the OpenVINS odometry into the odom -> base_link transform.

OpenVINS publishes /odomimu with header.frame_id "global" and child_frame_id
"imu". Its "imu" frame is the IMU's own axes, whose x axis points BACKWARD on
this robot, while base_link has x forward. base_link is at the same point as the
IMU and turned 180 degrees about z (see launch/lidar.launch.py), so:

    position(base_link in odom)    = position(imu in global)
    orientation(base_link in odom) = orientation(imu in global) * Rz(pi)

OpenVINS' "global" frame is used directly as "odom". Its origin is the IMU pose
at initialisation, so the robot starts near (0, 0). The starting heading is
chosen by OpenVINS; on the robot it was observed at about 0 deg (base_link x
along odom x) in two runs, but it is not guaranteed, so do not rely on it.

Also republishes the pose on /odom (nav_msgs/Odometry, odom -> base_link) for
nodes that want an Odometry message. The twist is left unset (covariance -1):
OpenVINS does not estimate angular velocity and its linear velocity frame is
not reliable enough to forward.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def rotate_pi_about_z(q):
    """Return q * Rz(pi) for a Hamilton quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    return (y, -x, w, -z)


class OdomTfNode(Node):
    def __init__(self):
        super().__init__('odom_tf_node')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('input_topic', '/odomimu')
        self.declare_parameter('publish_odom_topic', True)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        topic = self.get_parameter('input_topic').value

        self.tf_br = TransformBroadcaster(self)
        self.odom_pub = (self.create_publisher(Odometry, '/odom', 10)
                         if self.get_parameter('publish_odom_topic').value
                         else None)
        # OpenVINS publishes with a small reliable queue; match it loosely.
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Odometry, topic, self.on_odom, qos)
        self.get_logger().info(
            'Publishing %s -> %s from %s' % (self.odom_frame, self.base_frame, topic))

    def on_odom(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        qx, qy, qz, qw = rotate_pi_about_z((o.x, o.y, o.z, o.w))

        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = p.x
        tf.transform.translation.y = p.y
        tf.transform.translation.z = p.z
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_br.sendTransform(tf)

        if self.odom_pub is not None:
            out = Odometry()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = self.odom_frame
            out.child_frame_id = self.base_frame
            out.pose.pose.position = p
            out.pose.pose.orientation.x = qx
            out.pose.pose.orientation.y = qy
            out.pose.pose.orientation.z = qz
            out.pose.pose.orientation.w = qw
            out.pose.covariance = msg.pose.covariance
            out.twist.covariance[0] = -1.0   # twist not provided
            self.odom_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
