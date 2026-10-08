#!/usr/bin/env python3
"""
Node 3 — Web Stream Viewer
============================
Subscribes to both /video_raw and /video_processed and serves them
as MJPEG streams on a local web page, plus a project landing page.

Subscribes:
  /video_raw        (sensor_msgs/CompressedImage)
  /video_processed  (sensor_msgs/CompressedImage)

Endpoints:
  http://<pi-ip>:8080/                           — project landing page (architecture diagrams)
  http://<pi-ip>:8080/processed-feed             — side-by-side viewer
  http://<pi-ip>:8080/processed-feed/feed/raw    — raw MJPEG stream
  http://<pi-ip>:8080/processed-feed/feed/proc   — processed MJPEG stream
  http://<pi-ip>:8080/processed-feed/mode/<m>    — change processing mode (proxied to process_node param)
  http://<pi-ip>:8080/status                     — JSON health check

Parameters:
  port  (int)  8080
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.parameter import Parameter
from sensor_msgs.msg import CompressedImage

from flask import Flask, jsonify
import logging

from ugv1_controller.web.camera import create_camera_blueprint
from ugv1_controller.web.landing import landing_bp

# Silence Flask request logs — keep terminal clean for ROS2 logs
logging.getLogger('werkzeug').setLevel(logging.ERROR)

_IMAGE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class ViewerNode(Node):

    def __init__(self):
        super().__init__('viewer_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('port', 8080)
        self.port = self.get_parameter('port').value

        # ── Shared frame state ─────────────────────────────────────────────────
        self._raw_lock  = threading.Lock()
        self._proc_lock = threading.Lock()
        self._raw_frame  = None
        self._proc_frame = None

        # Track last-received time for health status
        self._raw_last_ts  = None
        self._proc_last_ts = None

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            CompressedImage, '/video_raw',
            self._raw_callback, _IMAGE_QOS
        )
        self.create_subscription(
            CompressedImage, '/video_processed',
            self._proc_callback, _IMAGE_QOS
        )

        # ── Flask app ─────────────────────────────────────────────────────────
        self._app = self._build_flask_app()
        flask_thread = threading.Thread(
            target=lambda: self._app.run(
                host='0.0.0.0', port=self.port, threaded=True
            ),
            daemon=True
        )
        flask_thread.start()

        self.get_logger().info(
            f'Viewer node started — http://0.0.0.0:{self.port}'
        )

    # ── ROS2 callbacks ────────────────────────────────────────────────────────

    def _raw_callback(self, msg: CompressedImage):
        data = bytes(msg.data)
        with self._raw_lock:
            self._raw_frame   = data
            self._raw_last_ts = time.time()

    def _proc_callback(self, msg: CompressedImage):
        data = bytes(msg.data)
        with self._proc_lock:
            self._proc_frame   = data
            self._proc_last_ts = time.time()

    # ── MJPEG generators ──────────────────────────────────────────────────────

    def gen_raw(self):
        while True:
            with self._raw_lock:
                frame = self._raw_frame
            if frame:
                yield (
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' +
                    frame + b'\r\n'
                )
            time.sleep(1 / 15)

    def gen_proc(self):
        while True:
            with self._proc_lock:
                frame = self._proc_frame
            if frame:
                yield (
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' +
                    frame + b'\r\n'
                )
            time.sleep(1 / 15)

    # ── Flask app factory ─────────────────────────────────────────────────────

    def _build_flask_app(self):
        # Pages live in ugv1_controller/web; this node only supplies the
        # frame generators and the health check.
        app  = Flask(__name__)
        node = self   # closure reference

        app.register_blueprint(landing_bp)
        app.register_blueprint(create_camera_blueprint(node))

        @app.route('/status')
        def status():
            now = time.time()

            def age(ts):
                return round(now - ts, 2) if ts else None
            return jsonify({
                'raw_frame_age_sec':  age(node._raw_last_ts),
                'proc_frame_age_sec': age(node._proc_last_ts),
                'raw_topic':          '/video_raw',
                'proc_topic':         '/video_processed',
                'port':               node.port,
            })

        return app


def main(args=None):
    rclpy.init(args=args)
    node = ViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
