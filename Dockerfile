FROM osrf/ros:humble-desktop

# For my own packages.
WORKDIR /ros2_ws
COPY . /ros2_ws/src/core2026
RUN set -eo pipefail \
    && source /opt/ros/humble/setup.bash \
    && rosdep update \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src --rosdistro humble -y \
    && colcon build --base-paths /ros2_ws \
    && rm -rf /var/lib/apt/lists/*

RUN apt update && apt install -y --no-install-recommends \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir pythonQwt

#ワークスペースのビルド
RUN . /opt/ros/humble/setup.bash && \
   colcon build --symlink-install

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