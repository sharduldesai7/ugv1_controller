#!/usr/bin/env python3
"""Prepare wheel odometry and IMU data for the EKF.

  /wheel_odom       -> /wheel_odom/conditioned   frames set, covariances set
  /imu/data_raw     -> /imu/conditioned          covariances set, gyro z bias removed

The wheel odometry comes from Gazebo in the simulation and from the encoder driver on the
rover. With publish_tf:=true the node also publishes odom -> base_link straight from the wheel
odometry (the "no EKF" option, for comparing against the EKF).

Time stamps are copied from the inputs, so the node works unchanged on simulated time.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from ugv1_controller.loc import conditioning as c


class SensorConditioner(Node):
    def __init__(self):
        super().__init__('sensor_conditioner')
        self.declare_parameter('wheel_cov_vx', 0.0025)         # (0.05 m/s)^2
        self.declare_parameter('wheel_cov_vyaw', 0.01)         # (0.1 rad/s)^2
        self.declare_parameter('imu_gyro_cov', 2.5e-5)         # (0.005 rad/s)^2
        self.declare_parameter('gyro_bias_calibration_s', 2.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', False)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.twist_cov = c.wheel_twist_covariance(self.get_parameter('wheel_cov_vx').value,
                                                  self.get_parameter('wheel_cov_vyaw').value)
        self.pose_cov = c.wheel_pose_covariance()
        self.gyro_cov = c.imu_angular_covariance(self.get_parameter('imu_gyro_cov').value)
        self.bias = c.GyroBias(self.get_parameter('gyro_bias_calibration_s').value)

        self.last_wheel = None          # (stamp s, vx, wz)
        self.logged_bias = False

        # Best-effort subscriptions connect to both reliable and best-effort publishers.
        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=200)
        out_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=50)
        self.wheel_pub = self.create_publisher(Odometry, c.WHEEL_OUT, out_qos)
        self.imu_pub = self.create_publisher(Imu, c.IMU_OUT, out_qos)
        self.tf_br = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Odometry, c.WHEEL_IN, self.on_wheel, sub_qos)
        self.create_subscription(Imu, c.IMU_IN, self.on_imu, sub_qos)
        self.get_logger().info(
            'Conditioning %s -> %s and %s -> %s (odom -> base_link TF: %s)'
            % (c.WHEEL_IN, c.WHEEL_OUT, c.IMU_IN, c.IMU_OUT, self.publish_tf))

    @staticmethod
    def _secs(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def on_wheel(self, msg):
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose = msg.pose.pose
        out.twist.twist = msg.twist.twist
        out.pose.covariance = self.pose_cov
        out.twist.covariance = self.twist_cov
        self.wheel_pub.publish(out)
        self.last_wheel = (self._secs(msg.header.stamp), msg.twist.twist.linear.x,
                           msg.twist.twist.angular.z)
        if self.tf_br is not None:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
            t.transform.rotation = q
            self.tf_br.sendTransform(t)

    def on_imu(self, msg):
        t = self._secs(msg.header.stamp)
        stationary = (self.last_wheel is not None and abs(t - self.last_wheel[0]) <= 0.5
                      and c.is_stationary(self.last_wheel[1], self.last_wheel[2]))
        self.bias.update(t, msg.angular_velocity.z, stationary)
        if self.bias.done and not self.logged_bias:
            self.logged_bias = True
            self.get_logger().info('Gyro z bias %.5f rad/s removed' % self.bias.bias)
        out = Imu()
        out.header = msg.header
        out.orientation_covariance[0] = -1.0          # no orientation estimate
        # Copy field by field: assigning a sub-message would share it with the input message.
        out.angular_velocity.x = msg.angular_velocity.x
        out.angular_velocity.y = msg.angular_velocity.y
        out.angular_velocity.z = self.bias.corrected(msg.angular_velocity.z)
        out.angular_velocity_covariance = self.gyro_cov
        out.linear_acceleration.x = msg.linear_acceleration.x
        out.linear_acceleration.y = msg.linear_acceleration.y
        out.linear_acceleration.z = msg.linear_acceleration.z
        out.linear_acceleration_covariance = c.imu_angular_covariance(0.1)
        self.imu_pub.publish(out)


def main():
    rclpy.init()
    node = SensorConditioner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
