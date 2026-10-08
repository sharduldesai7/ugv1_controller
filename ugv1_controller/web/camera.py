"""Camera viewer blueprint: MJPEG streams and processing-mode control.

The blueprint only needs an object exposing two methods, gen_raw() and
gen_proc(), that yield MJPEG multipart chunks (viewer_node provides them).
"""

import subprocess

from flask import Blueprint, Response, jsonify

VALID_MODES = ('raw', 'gray', 'edges', 'overlay')
_MJPEG_MIMETYPE = 'multipart/x-mixed-replace; boundary=frame'


def create_camera_blueprint(source):
    bp = Blueprint(
        'camera', __name__,
        url_prefix='/processed-feed',
        static_folder='static/camera',
        static_url_path='/static',
    )

    @bp.route('', strict_slashes=False)
    def index():
        return bp.send_static_file('index.html')

    @bp.route('/feed/raw')
    def feed_raw():
        return Response(source.gen_raw(), mimetype=_MJPEG_MIMETYPE)

    @bp.route('/feed/proc')
    def feed_proc():
        return Response(source.gen_proc(), mimetype=_MJPEG_MIMETYPE)

    @bp.route('/mode/<new_mode>')
    def set_mode(new_mode):
        if new_mode not in VALID_MODES:
            return jsonify(
                {'error': f'Invalid mode — choose from {VALID_MODES}'}), 400
        # ros2 param set is reliable across ROS 2 versions
        subprocess.Popen(
            ['ros2', 'param', 'set', '/process_node', 'mode', new_mode],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        return jsonify({'mode': new_mode})

    return bp
