FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Base tools for dependency resolution and workspace build.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# rosdep init may already be configured in some base images.
RUN rosdep init 2>/dev/null || true

# For my own packages.
WORKDIR /ros2_ws
# Copy only package manifests first (keep directory hierarchy) to maximize cache hit for dependency install.
COPY core_body_controller/package.xml /ros2_ws/src/core2026/core_body_controller/package.xml
COPY core_costmap_builder/package.xml /ros2_ws/src/core2026/core_costmap_builder/package.xml
COPY core_hardware/package.xml /ros2_ws/src/core2026/core_hardware/package.xml
COPY core_launch/package.xml /ros2_ws/src/core2026/core_launch/package.xml
COPY core_msgs/package.xml /ros2_ws/src/core2026/core_msgs/package.xml
COPY core_path_follower/package.xml /ros2_ws/src/core2026/core_path_follower/package.xml
COPY core_test/package.xml /ros2_ws/src/core2026/core_test/package.xml
COPY core_tools/package.xml /ros2_ws/src/core2026/core_tools/package.xml
RUN set -eo pipefail \
    && source /opt/ros/humble/setup.bash \
    && rosdep update \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src --rosdistro humble -y \
    && rm -rf /var/lib/apt/lists/*

# Then copy source code and build.
COPY . /ros2_ws/src/core2026
RUN set -eo pipefail \
    && source /opt/ros/humble/setup.bash \
    && colcon build --base-paths /ros2_ws

RUN apt update && apt install -y --no-install-recommends \
    python3-pip libgl1\
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir pythonQwt pyqt5-tools

#ワークスペースのビルド
# RUN . /opt/ros/humble/setup.bash && \
#    colcon build --symlink-install

RUN echo '. /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'export ROS2_WS=ros2_ws' >> /root/.bashrc && \
    echo 'function cw() { cd /${ROS2_WS}; }' >> /root/.bashrc && \
    echo 'function cs() { cd /${ROS2_WS}/src; }' >> /root/.bashrc && \
    echo 'function cb() { cd /${ROS2_WS}; if [ -z $1 ]; then colcon build --symlink-install; else colcon build --symlink-install --packages-select $1; fi; . install/setup.bash && cd -;}' >> /root/.bashrc && \
    echo 'function cbd() { cd /${ROS2_WS}; if [ -z $1 ]; then colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug; else colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select $1; fi; . install/setup.bash && cd -;}' >> /root/.bashrc && \
    echo 'function ws() { if [ $1 ]; then ROS2_WS=$1_ws&&echo "switch ${ROS2_WS}"&&. /${ROS2_WS}/install/setup.bash;fi;}' >> /root/.bashrc && \
    echo "alias cl='cw && rm -rf ./build ./install ./log && cd -'" >> /root/.bashrc && \
    echo '. /ros2_ws/install/setup.bash' >> /root/.bashrc && \
    echo '. /lib_ws/install/setup.bash' >> /root/.bashrc

# エントリポイント
CMD ["/bin/bash"]
