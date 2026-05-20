#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class subscriberNode(Node):

    def __init__(self):
        super().__init__("test_subscriber")
        self.data_sub_ = self.create_subscription(String, "/test_topic", self.subscribe_message, 10)
        self.data_pub_ = self.create_publisher(String, "/test_topic", 10)
        self.create_timer(1.0, self.publish_message)
        self.counter_ = 0

    def subscribe_message(self, msg: String):
        msg = String()
        self.get_logger().info("test_subscriber hears " + str(msg.data))
        try:
            self.counter_ = int(msg.data)
            self.counter_ += 1
        except:
            pass

    def publish_message(self):
        msg = String()
        msg.data = str(self.counter_)
        self.data_pub_.publish(msg)



def main(args = None):
    rclpy.init(args = args)
    firstSubscriberNode = subscriberNode()
    rclpy.spin(firstSubscriberNode)
    rclpy.shutdown()
