# Editor container for ros_map_edit.
#
# ROS 1 noetic + RViz + the vendored ros_map_edit package built into
# /catkin_ws/. Operator runs this on a workstation; G1 itself never
# uses this image.
#
# Build:    docker build -t map_edit_rviz:latest .
# Run once: see README.md "Build the editor container"

FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        ros-noetic-rviz \
        ros-noetic-map-server \
        ros-noetic-cv-bridge \
        ros-noetic-tf2-geometry-msgs \
        ros-noetic-interactive-markers \
        ros-noetic-pluginlib \
        qtbase5-dev \
        libjsoncpp-dev \
        libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws
COPY ros_map_edit /catkin_ws/src/ros_map_edit

RUN source /opt/ros/noetic/setup.bash && \
    catkin_make && \
    test -f /catkin_ws/devel/lib/libros_map_edit.so

# Auto-source for `docker exec -it ... bash` sessions. Non-interactive
# `docker exec` calls (like start_map_edit.sh) source explicitly.
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc && \
    echo 'source /catkin_ws/devel/setup.bash' >> /root/.bashrc

CMD ["sleep", "infinity"]
