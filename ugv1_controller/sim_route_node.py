#!/usr/bin/env python3
"""Simulation only: drive a fixed route through the warehouse aisles.

Follows waypoints using the simulator's ground-truth pose, so the path is the
same on every run whatever the simulation speed. Publishes /cmd_vel; do not run
it together with Drive Enable on the web page.

The rover first stands still for `start_delay` seconds (OpenVINS initialises
from rest), then turns on the spot to face each waypoint and drives to it. The
default route covers both aisles and both outer lanes and ends back at the
start, so the end-of-run position error is the drift over a closed loop.
"""

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .sim_route import DEFAULT_ROUTE, RouteFollower, yaw_of


class SimRouteNode(Node):
    def __init__(self):
        super().__init__('sim_route_node')
        self.declare_parameter('speed', 0.3)
        self.declare_parameter('turn_speed', 0.6)
        self.declare_parameter('start_delay', 5.0)
        self.declare_parameter('route', DEFAULT_ROUTE)
        route = [float(v) for v in self.get_parameter('route').value]
        self.follower = RouteFollower(
            route, float(self.get_parameter('speed').value),
            float(self.get_parameter('turn_speed').value),
            float(self.get_parameter('start_delay').value))
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_odom, 10)
        self._announced = False
        self.get_logger().info(
            f'Route of {len(self.follower.points)} waypoints; standing still for '
            f'{self.follower.start_delay:.0f} s of simulated time first')

    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose
        lin, ang = self.follower.step(t, p.position.x, p.position.y, yaw_of(p.orientation))
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = lin, ang
        self.pub.publish(cmd)
        if self.follower.done and not self._announced:
            self._announced = True
            self.get_logger().info('Route finished; stopped. Stop the bag recording now.')


def main(args=None):
    rclpy.init(args=args)
    node = SimRouteNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
