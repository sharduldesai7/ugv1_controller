"""Web interface for UGV1: a landing page and the first-person drive page.

Each page is a Flask blueprint with its own static folder (HTML, CSS, JS), so
the front end can be edited without touching any ROS node code. drive_state.py
holds the manual-drive rules and has no ROS imports, so it can be tested alone.
"""
