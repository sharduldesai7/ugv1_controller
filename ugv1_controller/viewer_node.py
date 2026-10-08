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

from flask import Flask, Response, jsonify
import logging

from ugv1_controller.landing_page import LANDING_HTML

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

    def _gen_raw(self):
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

    def _gen_proc(self):
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
        app  = Flask(__name__)
        node = self   # closure reference

        @app.route('/processed-feed/feed/raw')
        def feed_raw():
            return Response(
                node._gen_raw(),
                mimetype='multipart/x-mixed-replace; boundary=frame'
            )

        @app.route('/processed-feed/feed/proc')
        def feed_proc():
            return Response(
                node._gen_proc(),
                mimetype='multipart/x-mixed-replace; boundary=frame'
            )

        @app.route('/processed-feed/mode/<new_mode>')
        def set_mode(new_mode):
            valid = ('raw', 'gray', 'edges', 'overlay')
            if new_mode not in valid:
                return jsonify({'error': f'Invalid mode — choose from {valid}'}), 400
            # Set parameter on the process_node via ROS2 parameter service
            try:
                client = node.create_client(
                    rclpy.parameter.parameter_service.SetParameters,
                    '/process_node/set_parameters'
                )
            except Exception:
                pass
            # Simpler: use subprocess ros2 param set (reliable cross-version)
            import subprocess
            subprocess.Popen(
                ['ros2', 'param', 'set', '/process_node', 'mode', new_mode],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
            return jsonify({'mode': new_mode})


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

        @app.route('/')
        def landing():
            return LANDING_HTML

        @app.route('/processed-feed', strict_slashes=False)
        def processed_feed():
            return _HTML

        return app


# ── HTML viewer ───────────────────────────────────────────────────────────────

_HTML = '''<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Rover Vision</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Space+Mono:wght@400;700&display=swap');
  *,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
  :root{
    --bg:#0a0c0f;--surface:#111418;--border:#1e2530;
    --accent:#00e5a0;--text:#e8edf2;--muted:#4a5568;
    --mono:'Space Mono',monospace;
  }
  body{background:var(--bg);color:var(--text);font-family:var(--mono);
       min-height:100vh;display:flex;flex-direction:column}

  header{border-bottom:1px solid var(--border);padding:14px 24px;
         display:flex;align-items:center;gap:12px}
  .dot{width:8px;height:8px;border-radius:50%;background:var(--accent);
       box-shadow:0 0 8px var(--accent);animation:blink 2s ease-in-out infinite}
  @keyframes blink{0%,100%{opacity:1}50%{opacity:.3}}
  h1{font-size:12px;font-weight:700;letter-spacing:.14em;text-transform:uppercase}
  .badge{margin-left:auto;font-size:10px;color:var(--accent);
         border:1px solid var(--accent);border-radius:3px;padding:2px 8px}

  main{flex:1;display:flex;flex-direction:column;align-items:center;
       padding:24px 16px;gap:20px}

  /* Side by side feeds */
  .feeds{display:grid;grid-template-columns:1fr 1fr;gap:16px;
         width:100%;max-width:1100px}
  @media(max-width:700px){.feeds{grid-template-columns:1fr}}

  .feed-card{display:flex;flex-direction:column;gap:8px}
  .feed-label{font-size:9px;letter-spacing:.16em;text-transform:uppercase;
              color:var(--muted);padding-left:2px}
  .feed-label span{color:var(--accent)}

  .feed-wrap{position:relative;border:1px solid var(--border);border-radius:6px;
             overflow:hidden;background:#000;width:100%}
  .feed-wrap img{width:100%;display:block}
  .corner{position:absolute;width:12px;height:12px;
          border-color:var(--accent);border-style:solid;opacity:.5;z-index:2}
  .tl{top:6px;left:6px;border-width:1.5px 0 0 1.5px}
  .tr{top:6px;right:6px;border-width:1.5px 1.5px 0 0}
  .bl{bottom:6px;left:6px;border-width:0 0 1.5px 1.5px}
  .br{bottom:6px;right:6px;border-width:0 1.5px 1.5px 0}
  .overlay-tag{position:absolute;top:8px;left:50%;transform:translateX(-50%);
               font-size:9px;letter-spacing:.14em;color:var(--accent);
               opacity:.6;z-index:2;text-transform:uppercase;white-space:nowrap}
  .mode-tag{position:absolute;bottom:8px;right:10px;font-size:9px;
            letter-spacing:.12em;color:var(--accent);opacity:.7;
            z-index:2;text-transform:uppercase}

  /* Controls */
  .controls{display:flex;gap:10px;flex-wrap:wrap;justify-content:center}
  button{font-family:var(--mono);font-size:11px;letter-spacing:.1em;
         text-transform:uppercase;padding:8px 20px;border-radius:4px;
         border:1px solid var(--border);background:var(--surface);
         color:var(--muted);cursor:pointer;transition:all .15s}
  button:hover{border-color:var(--accent);color:var(--accent)}
  button.active{border-color:var(--accent);color:var(--accent);
                background:rgba(0,229,160,.08)}

  .meta{font-size:10px;color:var(--muted);display:flex;gap:20px;
        flex-wrap:wrap;justify-content:center;letter-spacing:.05em}
  .meta span{color:var(--text)}



  footer{border-top:1px solid var(--border);padding:12px 24px;font-size:10px;
         color:var(--muted);letter-spacing:.07em;display:flex;gap:20px;flex-wrap:wrap}
  footer a{color:var(--accent);text-decoration:none}
</style>
</head>
<body>

<header>
  <div class="dot"></div>
  <h1>Rover Vision Pipeline</h1>
  <div class="badge">LIVE</div>
</header>

<main>

  <div class="feeds">
    <!-- Raw feed -->
    <div class="feed-card">
      <div class="feed-label">Topic: <span>/video_raw</span></div>
      <div class="feed-wrap">
        <div class="corner tl"></div><div class="corner tr"></div>
        <div class="corner bl"></div><div class="corner br"></div>
        <div class="overlay-tag">RAW · OV5647</div>
        <img src="/processed-feed/feed/raw" id="img-raw" alt="Raw feed">
      </div>
    </div>

    <!-- Processed feed -->
    <div class="feed-card">
      <div class="feed-label">Topic: <span>/video_processed</span></div>
      <div class="feed-wrap">
        <div class="corner tl"></div><div class="corner tr"></div>
        <div class="corner bl"></div><div class="corner br"></div>
        <div class="overlay-tag">PROCESSED</div>
        <div class="mode-tag" id="mode-tag">EDGES</div>
        <img src="/processed-feed/feed/proc" id="img-proc" alt="Processed feed">
      </div>
    </div>
  </div>

  <!-- Mode controls -->
  <div class="controls">
    <button onclick="setMode('raw')"     id="btn-raw">Colour</button>
    <button onclick="setMode('gray')"    id="btn-gray">Greyscale</button>
    <button onclick="setMode('edges')"   id="btn-edges" class="active">Edges</button>
    <button onclick="setMode('overlay')" id="btn-overlay">Overlay</button>
  </div>


  <div class="meta">
    capture: <span>/video_raw</span> &nbsp;·&nbsp;
    processed: <span>/video_processed</span> &nbsp;·&nbsp;
    raw stream: <span><a href="/processed-feed/feed/raw">/processed-feed/feed/raw</a></span> &nbsp;·&nbsp;
    proc stream: <span><a href="/processed-feed/feed/proc">/processed-feed/feed/proc</a></span>
  </div>

</main>

<footer>
  <span>3-NODE ROS2 PIPELINE</span>
  <a href="/">HOME</a>
  <a href="/processed-feed/feed/raw">RAW STREAM</a>
  <a href="/processed-feed/feed/proc">PROCESSED STREAM</a>
  <a href="/status">STATUS</a>
</footer>

<script>
  let currentMode = 'edges';
  const modeLabels = {raw:'COLOUR', gray:'GREYSCALE', edges:'EDGES', overlay:'OVERLAY'};

  function setMode(mode) {
    fetch('/processed-feed/mode/' + mode)
      .then(r => r.json())
      .then(() => {
        currentMode = mode;
        document.getElementById('mode-tag').textContent = modeLabels[mode] || mode.toUpperCase();
        ['raw','gray','edges','overlay'].forEach(m => {
          document.getElementById('btn-' + m).classList.toggle('active', m === mode);
        });
        // Force reload of processed feed to clear stale frame
        setTimeout(() => {
          const img = document.getElementById('img-proc');
          img.src = '/processed-feed/feed/proc?' + Date.now();
        }, 300);
      });
  }

</script>
</body>
</html>'''


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
