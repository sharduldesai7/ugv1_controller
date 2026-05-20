#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16

class nodeB(Node):

    def __init__(self):
        super().__init__("nodeB")
        self.data_sub_ = self.create_subscription(Int16, "/topic_A", self.subscribe_message, 10)
        self.data_pub_ = self.create_publisher(Int16, "/topic_B", 10)
        self.create_timer(1.0, self.publish_message)
        self.counter_ = 0

    def subscribe_message(self, msg: Int16):
        msg = Int16()
        self.get_logger().info("nodeB hears " + str(msg.data))
        try:
            self.counter_ = msg.data
            self.counter_ += 1
            msg.data = self.counter_
            self.data_pub_.publish(msg)
        except:
            self.get_logger().info("Encountered error")

    def publish_message(self):
        msg = Int16()
        msg.data = self.counter_
        self.data_pub_.publish(msg)
        self.get_logger().info(str(msg.data))


def main(args = None):
    rclpy.init(args = args)
    firstSubscriberNode = nodeB()
    rclpy.spin(firstSubscriberNode)
    rclpy.shutdown()
