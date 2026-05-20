#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class publisherNode(Node):

    def __init__(self):
        super().__init__("test_publisher") #Argument is the node name
        self.data_pub_ = self.create_publisher(String, "/test_topic", 10) #Create a publisher (Data type, Topic name, Queue size)
        self.create_timer(1.0, self.publish_message)

        #self.data_sub_ = self.create_subscription(String, "/test_topic", self.subscribe_message, 10)
        self.counter_ = 0

    def publish_message(self):
        msg = String()
        self.counter_ += 1
        msg.data = str(self.counter_)
        self.data_pub_.publish(msg)

    def subscribe_message(self, msg: String):
        msg = String()
        self.get_logger().info("test_publisher hears " + str(msg.data))
        try:
            self.counter_ = int(msg.data)
            self.counter_ += 1

        except:
            pass


def main(args = None):
    rclpy.init(args = args)
    firstPublisherNode = publisherNode()
    rclpy.spin(firstPublisherNode)
    rclpy.shutdown()
