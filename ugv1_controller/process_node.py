#!/usr/bin/env python3
"""
Node 2 — Image Processing
===========================
Subscribes to /video_raw, applies OpenCV processing, publishes to /video_processed.

Subscribes:
  /video_raw        (sensor_msgs/CompressedImage)

Publishes:
  /video_processed  (sensor_msgs/CompressedImage)

Parameters:
  mode          (str)   'edges'  — raw | gray | edges | overlay
  canny_low     (int)   50
  canny_high    (int)   150
  jpeg_quality  (int)   80

Mode can be changed at runtime:
  ros2 param set /process_node mode overlay
"""

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult


_IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class ProcessNode(Node):

    VALID_MODES = ('raw', 'gray', 'edges', 'overlay')

    def __init__(self):
        super().__init__('process_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter(
            'mode', 'edges',
            ParameterDescriptor(description='Processing mode: raw|gray|edges|overlay')
        )
        self.declare_parameter('canny_low',    50)
        self.declare_parameter('canny_high',   150)
        self.declare_parameter('jpeg_quality', 80)

        self._read_params()

        # Allow live parameter updates
        self.add_on_set_parameters_callback(self._on_param_change)

        # ── Subscriber ────────────────────────────────────────────────────────
        self.sub = self.create_subscription(
            CompressedImage,
            '/video_raw',
            self._image_callback,
            _IMAGE_QOS,
        )

        # ── Publisher ─────────────────────────────────────────────────────────
        self.pub = self.create_publisher(
            CompressedImage, '/video_processed', _IMAGE_QOS
        )

        # ── Frame counter for logging ─────────────────────────────────────────
        self._frame_count = 0
        self._last_log    = self.get_clock().now()

        self.get_logger().info(
            f'Process node started — mode={self.mode}, '
            f'canny=[{self.canny_low},{self.canny_high}]'
        )

    # ── Parameter helpers ─────────────────────────────────────────────────────

    def _read_params(self):
        self.mode         = self.get_parameter('mode').value
        self.canny_low    = self.get_parameter('canny_low').value
        self.canny_high   = self.get_parameter('canny_high').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

    def _on_param_change(self, params):
        for p in params:
            if p.name == 'mode' and p.value not in self.VALID_MODES:
                return SetParametersResult(
                    successful=False,
                    reason=f'mode must be one of {self.VALID_MODES}'
                )
        self._read_params()
        self.get_logger().info(f'Parameters updated — mode={self.mode}')
        return SetParametersResult(successful=True)

    # ── Subscription callback ─────────────────────────────────────────────────

    def _image_callback(self, msg: CompressedImage):
        # Decode JPEG bytes → numpy BGR
        arr   = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('Failed to decode incoming frame')
            return

        # Apply selected processing mode
        try:
            processed = self._apply_mode(frame, self.mode)
        except Exception as e:
            self.get_logger().error(f'Processing error: {e}')
            processed = frame

        # Re-encode to JPEG
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        ok, jpeg_buf  = cv2.imencode('.jpg', processed, encode_params)
        if not ok:
            self.get_logger().warn('JPEG encode failed')
            return

        # Publish
        out_msg = CompressedImage()
        out_msg.header = Header()
        out_msg.header.stamp    = msg.header.stamp   # preserve original timestamp
        out_msg.header.frame_id = msg.header.frame_id
        out_msg.format = 'jpeg'
        out_msg.data   = list(jpeg_buf.tobytes())
        self.pub.publish(out_msg)

        # Log throughput every 5 seconds
        self._frame_count += 1
        now     = self.get_clock().now()
        elapsed = (now - self._last_log).nanoseconds / 1e9
        if elapsed >= 5.0:
            fps = self._frame_count / elapsed
            self.get_logger().info(
                f'Processing {fps:.1f} fps — mode={self.mode}'
            )
            self._frame_count = 0
            self._last_log    = now

    # ── OpenCV processing modes ───────────────────────────────────────────────

    def _apply_mode(self, frame_bgr: np.ndarray, mode: str) -> np.ndarray:
        """
        Apply the selected processing mode.
        All operations work on the frame in-place or return a new array.
        """
        if mode == 'raw':
            return frame_bgr

        # Grayscale + blur — shared by gray/edges/overlay
        gray    = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)

        if mode == 'gray':
            return cv2.cvtColor(blurred, cv2.COLOR_GRAY2BGR)

        # Canny edge detection
        edges = cv2.Canny(blurred, self.canny_low, self.canny_high)

        if mode == 'edges':
            # White edges on black background
            return cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        if mode == 'overlay':
            # Green edges composited over colour frame
            result = frame_bgr.copy()
            result[edges > 0] = (0, 255, 0)
            return result

        return frame_bgr   # fallback


def main(args=None):
    rclpy.init(args=args)
    node = ProcessNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
