#!/usr/bin/env python3
"""Simulation only: turn the simulated raw camera image into the JPEG topic.

The real capture_node publishes /video_raw (CompressedImage) for the web page
and /video_raw_raw (Image) for OpenVINS. In the simulation Gazebo produces the
raw image, and this node produces /video_raw from it so web_node works
unchanged.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image

_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                  history=HistoryPolicy.KEEP_LAST, depth=1)


def image_to_bgr(msg):
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding == 'mono8':
        gray = data.reshape(msg.height, msg.step)[:, :msg.width]
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if msg.encoding in ('rgb8', 'bgr8'):
        img = data.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1] if msg.encoding == 'rgb8' else img
    raise ValueError(f'unsupported encoding {msg.encoding}')


class SimCameraRelay(Node):
    def __init__(self):
        super().__init__('sim_camera_relay')
        self.declare_parameter('quality', 80)
        self.quality = int(self.get_parameter('quality').value)
        self.pub = self.create_publisher(CompressedImage, '/video_raw', _QOS)
        self.create_subscription(Image, '/video_raw_raw', self.on_image, _QOS)

    def on_image(self, msg):
        try:
            bgr = image_to_bgr(msg)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return
        ok, jpeg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data = jpeg.tobytes()
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SimCameraRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
