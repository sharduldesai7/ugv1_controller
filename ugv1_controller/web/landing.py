"""Landing page blueprint: project overview and architecture diagrams.

Serves "/" and has no dependency on ROS, so it can be reused or tested alone.
"""

from flask import Blueprint

landing_bp = Blueprint(
    'landing', __name__,
    static_folder='static/landing',
    static_url_path='/landing-static',
)


@landing_bp.route('/')
def index():
    return landing_bp.send_static_file('index.html')
