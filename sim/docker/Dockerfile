# UGV1 simulation container: ROS 2 Jazzy + Gazebo Harmonic, native arm64 on an Apple-silicon Mac,
# with a VNC desktop for the Gazebo window. No GPU in the container, so OpenGL is software
# (llvmpipe), which is what Gazebo's renderer needs when the host offers no usable GPU.
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-tf2-ros ros-jazzy-rviz2 \
      ros-jazzy-rosbag2-storage-default-plugins \
      python3-colcon-common-extensions python3-opencv python3-flask python3-yaml python3-numpy \
      tigervnc-standalone-server tigervnc-common openbox xterm dbus-x11 \
      xvfb mesa-utils libgl1-mesa-dri \
      git ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# vncpasswd is in its own package; kept as a separate layer so the big install above stays cached
RUN apt-get update && apt-get install -y --no-install-recommends tigervnc-tools && rm -rf /var/lib/apt/lists/*

ENV LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe DISPLAY=:1
RUN mkdir -p /root/.vnc && echo "password" | vncpasswd -f > /root/.vnc/passwd && chmod 600 /root/.vnc/passwd

COPY start.sh /start.sh
RUN chmod +x /start.sh && echo "source /opt/ros/jazzy/setup.bash; [ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/ros2_ws
EXPOSE 5901 8080
CMD ["/start.sh"]
