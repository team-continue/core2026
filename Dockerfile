FROM osrf/ros:humble-desktop-full

# コマンドの追加
RUN echo '. /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'export ROS2_WS=ros2_ws' >> /root/.bashrc && \
    echo 'function cw() { cd /${ROS2_WS}; }' >> /root/.bashrc && \
    echo 'function cs() { cd /${ROS2_WS}/src; }' >> /root/.bashrc && \
    echo 'function cb() { cd /${ROS2_WS}; if [ -z $1 ]; then colcon build --symlink-install; else colcon build --symlink-install --packages-select $1; fi; . install/setup.bash && cd -;}' >> /root/.bashrc && \
    echo 'function cbd() { cd /${ROS2_WS}; if [ -z $1 ]; then colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug; else colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select $1; fi; . install/setup.bash && cd -;}' >> /root/.bashrc && \
    echo 'function ws() { if [ $1 ]; then ROS2_WS=$1_ws&&echo "switch ${ROS2_WS}"&&. /${ROS2_WS}/install/setup.bash;fi;}' >> /root/.bashrc && \
    echo "alias cl='cw && rm -rf ./build ./install ./log && cd -'" >> /root/.bashrc && \
    echo '. /ros2_ws/install/setup.bash' >> /root/.bashrc && \
    mkdir -p /ros2_ws/src

# ワークスペースの作成とパッケージのコピー
WORKDIR /ros2_ws

COPY . /ros2_ws/src/

# 依存関係のインストール
SHELL ["/bin/bash", "-c"]
RUN . /opt/ros/humble/setup.bash && \
    apt update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

#ワークスペースのビルド
#RUN . /opt/ros/humble/setup.bash && \
#    colcon build --symlink-install

# エントリポイント
CMD ["/bin/bash"]