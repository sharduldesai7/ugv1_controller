"""Drive page blueprint: raw camera feed plus manual control, served at /drive.

The blueprint needs two things:
  source      an object with gen_raw() yielding MJPEG multipart chunks
  controller  a DriveController (see drive_state.py)

All control endpoints take JSON, identify the browser tab by a random
"client" string it generates on page load, and answer with the new state.
"""

from flask import Blueprint, Response, jsonify, request

_MJPEG_MIMETYPE = 'multipart/x-mixed-replace; boundary=frame'


def _client_id(data):
    cid = data.get('client') if isinstance(data, dict) else None
    if isinstance(cid, str) and 0 < len(cid) <= 64:
        return cid
    return None


def create_drive_blueprint(source, controller):
    bp = Blueprint(
        'drive', __name__,
        url_prefix='/drive',
        static_folder='static/drive',
        static_url_path='/static',
    )

    def reply(ok, reason, client):
        body = controller.snapshot(client)
        body['ok'] = ok
        body['error'] = reason
        return jsonify(body), (200 if ok else 409)

    @bp.route('', strict_slashes=False)
    def index():
        return bp.send_static_file('index.html')

    @bp.route('/feed/raw')
    def feed_raw():
        return Response(source.gen_raw(), mimetype=_MJPEG_MIMETYPE)

    @bp.route('/api/state')
    def state():
        client = request.args.get('client', '')[:64]
        body = controller.snapshot(client)
        body['ok'] = True
        body['error'] = None
        return jsonify(body)

    @bp.route('/api/enable', methods=['POST'])
    def enable():
        data = request.get_json(silent=True) or {}
        client = _client_id(data)
        if client is None:
            return jsonify({'ok': False, 'error': 'bad_request'}), 400
        ok, reason = controller.enable(client)
        return reply(ok, reason, client)

    @bp.route('/api/disable', methods=['POST'])
    def disable():
        data = request.get_json(silent=True) or {}
        client = _client_id(data)
        if client is None:
            return jsonify({'ok': False, 'error': 'bad_request'}), 400
        ok, reason = controller.disable(client)
        return reply(ok, reason, client)

    @bp.route('/api/cmd', methods=['POST'])
    def cmd():
        data = request.get_json(silent=True) or {}
        client = _client_id(data)
        try:
            x = float(data.get('x', 0))
            th = float(data.get('th', 0))
        except (TypeError, ValueError):
            client = None
        if client is None:
            return jsonify({'ok': False, 'error': 'bad_request'}), 400
        ok, reason = controller.command(client, x, th)
        return reply(ok, reason, client)

    @bp.route('/api/speed', methods=['POST'])
    def speed():
        data = request.get_json(silent=True) or {}
        client = _client_id(data)
        target = data.get('target')
        direction = data.get('direction')
        if client is None or target not in ('both', 'lin', 'turn') \
                or direction not in (1, -1):
            return jsonify({'ok': False, 'error': 'bad_request'}), 400
        ok, reason = controller.adjust_speed(client, target, direction)
        return reply(ok, reason, client)

    return bp
