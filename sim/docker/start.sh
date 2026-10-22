#!/bin/bash
# Start a VNC X server (:1) with a minimal window manager, then keep the container alive.
set -e
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1
Xvnc :1 -geometry 1600x900 -depth 24 -SecurityTypes VncAuth -rfbauth /root/.vnc/passwd -rfbport 5901 &
sleep 2
DISPLAY=:1 openbox &
echo "VNC desktop ready on port 5901 (password: password). Open a shell with: docker exec -it ugv1-sim bash"
wait
