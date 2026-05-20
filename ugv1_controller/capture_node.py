#!/usr/bin/env python3
"""
Node 1 — Image Capture
========================
Captures frames from the OV5647 Pi camera via rpicam-vid and publishes
raw compressed images to /video_raw.

Publishes:
  /video_raw      (sensor_msgs/CompressedImage)  JPEG frames at TARGET_FPS
  /video_raw_raw  (sensor_msgs/Image)             Raw BGR frames (if publish_raw=True)

Parameters:
  width       (int)   640
  height      (int)   480
  fps         (int)   30
  target_fps  (int)   15   — publish rate (decoupled from capture rate)
  vflip       (bool)  True
  hflip       (bool)  True
  publish_raw (bool)  False — if True, also publish raw Image for OpenVINS
"""

import subprocess
import threading
import queue
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header
import cv2
import numpy as np

_IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

_RAW_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

class CaptureNode(Node):

    def __init__(self):
        super().__init__('capture_node')

        self.declare_parameter('width',       640)
        self.declare_parameter('height',      480)
        self.declare_parameter('fps',         30)
        self.declare_parameter('target_fps',  15)
        self.declare_parameter('vflip',       True)
        self.declare_parameter('hflip',       True)
        self.declare_parameter('publish_raw', False)

        self.width       = self.get_parameter('width').value
        self.height      = self.get_parameter('height').value
        self.fps         = self.get_parameter('fps').value
        self.target_fps  = self.get_parameter('target_fps').value
        self.vflip       = self.get_parameter('vflip').value
        self.hflip       = self.get_parameter('hflip').value
        self.publish_raw = self.get_parameter('publish_raw').value

        # Compressed publisher (always on)
        self.pub_compressed = self.create_publisher(
            CompressedImage, '/video_raw', _IMAGE_QOS
        )

        # Raw publisher (only if publish_raw=True)
        self.pub_raw = None
        if self.publish_raw:
            self.pub_raw = self.create_publisher(
                Image, '/video_raw_raw', _RAW_QOS
            )
            self.get_logger().info('Raw image publishing enabled on /video_raw_raw')

        self._queue = queue.Queue(maxsize=2)
        self._running = True
        self._capture_thread = threading.Thread(
            target=self._capture_loop, daemon=True
        )
        self._capture_thread.start()

        self.create_timer(1.0 / self.target_fps, self._publish_callback)

        self.get_logger().info(
            f'Capture node started — '
            f'{self.width}x{self.height} @ {self.fps}fps '
            f'publishing at {self.target_fps}fps'
        )

    def _build_cmd(self):
        cmd = [
            'rpicam-vid',
            '--codec',     'mjpeg',
            '--width',     str(self.width),
            '--height',    str(self.height),
            '--framerate', str(self.fps),
            '--timeout',   '0',
            '--nopreview',
            '-o', '-',
        ]
        if self.vflip:
            cmd.append('--vflip')
        if self.hflip:
            cmd.append('--hflip')
        return cmd

    def _capture_loop(self):
        while self._running:
            self.get_logger().info('[capture] Starting rpicam-vid...')
            try:
                proc = subprocess.Popen(
                    self._build_cmd(),
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    bufsize=0,
                )
            except FileNotFoundError:
                self.get_logger().error('rpicam-vid not found')
                time.sleep(5)
                continue

            buf = b''
            try:
                while self._running:
                    chunk = proc.stdout.read(8192)
                    if not chunk:
                        self.get_logger().warn('[capture] Stream ended, restarting...')
                        break
                    buf += chunk

                    while True:
                        start = buf.find(b'\xff\xd8')
                        end   = buf.find(b'\xff\xd9')
                        if start == -1 or end == -1 or end <= start:
                            break
                        jpeg = buf[start:end + 2]
                        buf  = buf[end + 2:]
                        try:
                            self._queue.put_nowait(jpeg)
                        except queue.Full:
                            pass

                    if len(buf) > 4 * 1024 * 1024:
                        self.get_logger().warn('[capture] Buffer overflow — resetting')
                        buf = b''

            except Exception as e:
                self.get_logger().error(f'[capture] Exception: {e}')
            finally:
                proc.terminate()
                proc.wait()

            if self._running:
                time.sleep(1)

    def _publish_callback(self):
        try:
            jpeg = self._queue.get_nowait()
        except queue.Empty:
            return

        stamp = self.get_clock().now().to_msg()

        # Always publish compressed
        compressed_msg = CompressedImage()
        compressed_msg.header.stamp    = stamp
        compressed_msg.header.frame_id = 'camera_frame'
        compressed_msg.format = 'jpeg'
        compressed_msg.data   = list(jpeg)
        self.pub_compressed.publish(compressed_msg)

        # Optionally publish raw
        if self.pub_raw is not None:
            np_arr = np.frombuffer(jpeg, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                return
            raw_msg = Image()
            raw_msg.header.stamp    = stamp
            raw_msg.header.frame_id = 'camera_frame'
            raw_msg.height   = frame.shape[0]
            raw_msg.width    = frame.shape[1]
            raw_msg.encoding = 'bgr8'
            raw_msg.step     = frame.shape[1] * 3
            raw_msg.data     = frame.tobytes()
            self.pub_raw.publish(raw_msg)

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
