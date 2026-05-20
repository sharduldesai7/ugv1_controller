#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16

class nodeA(Node):

    def __init__(self):
        super().__init__("nodeA") #Argument is the node name
        self.data_pub_ = self.create_publisher(Int16, "/topic_A", 10) #Create a publisher (Data type, Topic name, Queue size)
        self.create_timer(1.0, self.publish_message)
        self.data_sub_ = self.create_subscription(Int16, "/topic_B", self.subscribe_message, 10)
        self.counter_ = 0
        #self.msginit_ = Int16()
        #self.msginit_.data = self.counter_
        #self.data_pub_.publish(self.msginit_)

    def publish_message(self):
        msg = Int16()
        #self.counter_ += 1
        msg.data = self.counter_
        self.data_pub_.publish(msg)
        #self.get_logger().info("nodeA sends " + msg.data)
        self.get_logger().info(str(msg.data))

    def subscribe_message(self, msg: Int16):
        msg = Int16()
        self.get_logger().info("nodeA hears " + str(msg.data))
        try:
            self.counter_ = msg.data
            self.counter_ += 1
            msg.data = self.counter_
            self.get_logger().info(str(self.counter_))
            self.data_pub_.publish(msg)
        except:
            self.get_logger().info("Encountered error")


def main(args = None):
    rclpy.init(args = args)
    firstPublisherNode = nodeA()
    rclpy.spin(firstPublisherNode)
    rclpy.shutdown()
