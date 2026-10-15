#!/usr/bin/env python3
"""
Web node
========
Serves the UGV1 web interface and is the manual-drive source for /cmd_vel.

Subscribes:
  /video_raw        (sensor_msgs/CompressedImage)  camera feed for the page
  /imu/data_raw     (sensor_msgs/Imu)              to confirm the rover is still

Publishes:
  /cmd_vel          (geometry_msgs/Twist)          only while Drive Enable is on

Endpoints:
  http://<pi-ip>:8080/                landing page (architecture diagrams)
  http://<pi-ip>:8080/drive           first-person drive page (raw feed + controls)
  http://<pi-ip>:8080/drive/feed/raw  raw MJPEG stream
  http://<pi-ip>:8080/drive/api/...   drive control API (see web/drive.py)
  http://<pi-ip>:8080/status          JSON health check

Parameters:
  port               (int)    8080
  lin_speed          (float)  0.5   starting linear speed limit, m/s
  turn_speed         (float)  1.0   starting turn speed limit, rad/s
  cmd_timeout        (float)  0.3   a drive command older than this counts as zero, s
  link_timeout       (float)  2.0   driver silent this long releases Drive Enable, s
  still_window       (float)  0.5   rover must be at rest this long to switch off, s
  still_gyro_range   (float)  0.08  max gyro spread over that window, rad/s
  still_accel_range  (float)  0.5   max accel spread over that window, m/s^2
"""

import logging
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Imu
from geometry_msgs.msg import Twist

from flask import Flask, jsonify

from ugv1_controller.web.drive import create_drive_blueprint
from ugv1_controller.web.drive_state import DriveController
from ugv1_controller.web.landing import landing_bp

# Silence Flask request logs — keep terminal clean for ROS2 logs
logging.getLogger('werkzeug').setLevel(logging.ERROR)

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

_TICK_HZ = 20.0


class WebNode(Node):

    def __init__(self):
        super().__init__('web_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('port', 8080)
        self.declare_parameter('lin_speed', 0.5)
        self.declare_parameter('turn_speed', 1.0)
        self.declare_parameter('cmd_timeout', 0.3)
        self.declare_parameter('link_timeout', 2.0)
        self.declare_parameter('still_window', 0.5)
        self.declare_parameter('still_gyro_range', 0.08)
        self.declare_parameter('still_accel_range', 0.5)
        self.port = self.get_parameter('port').value

        # ── Camera frame state ────────────────────────────────────────────────
        self._raw_lock = threading.Lock()
        self._raw_frame = None
        self._raw_last_ts = None

        # ── Manual drive ──────────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        def p(name):
            return self.get_parameter(name).value

        self.drive = DriveController(
            publish=self._publish_twist,
            lin=p('lin_speed'), turn=p('turn_speed'),
            cmd_timeout=p('cmd_timeout'), link_timeout=p('link_timeout'),
            still_window=p('still_window'),
            still_gyro_range=p('still_gyro_range'),
            still_accel_range=p('still_accel_range'),
        )

        # ── ROS interfaces ────────────────────────────────────────────────────
        self.create_subscription(
            CompressedImage, '/video_raw', self._raw_callback, _SENSOR_QOS)
        self.create_subscription(
            Imu, '/imu/data_raw', self._imu_callback, _SENSOR_QOS)
        self.create_timer(1.0 / _TICK_HZ, self.drive.tick)

        # ── Flask app ─────────────────────────────────────────────────────────
        self._app = self._build_flask_app()
        flask_thread = threading.Thread(
            target=lambda: self._app.run(
                host='0.0.0.0', port=self.port, threaded=True),
            daemon=True
        )
        flask_thread.start()

        self.get_logger().info(f'Web node started — http://0.0.0.0:{self.port}')

    # ── ROS2 callbacks ────────────────────────────────────────────────────────

    def _raw_callback(self, msg: CompressedImage):
        data = bytes(msg.data)
        with self._raw_lock:
            self._raw_frame = data
            self._raw_last_ts = time.time()

    def _imu_callback(self, msg: Imu):
        w, a = msg.angular_velocity, msg.linear_acceleration
        self.drive.imu_sample(w.x, w.y, w.z, a.x, a.y, a.z)

    def _publish_twist(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._cmd_pub.publish(msg)

    # ── MJPEG generator ───────────────────────────────────────────────────────

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

    # ── Flask app factory ─────────────────────────────────────────────────────

    def _build_flask_app(self):
        # Pages live in ugv1_controller/web; this node supplies the frame
        # generator, the drive controller and the health check.
        app = Flask(__name__)
        node = self

        app.register_blueprint(landing_bp)
        app.register_blueprint(create_drive_blueprint(node, node.drive))

        @app.route('/status')
        def status():
            age = (round(time.time() - node._raw_last_ts, 2)
                   if node._raw_last_ts else None)
            return jsonify({
                'raw_frame_age_sec': age,
                'raw_topic': '/video_raw',
                'port': node.port,
                'drive': node.drive.snapshot(None),
            })

        return app


def main(args=None):
    rclpy.init(args=args)
    node = WebNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
