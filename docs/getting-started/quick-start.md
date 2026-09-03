# クイックスタート

## 前提条件

- ROS 2 Humble
- Ubuntu 22.04

## ビルド

```bash
cd ~/core_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -i -y
colcon build --symlink-install
source install/setup.bash
```

## 起動方法

実機システムは、`core_launch` が提供する3つの統合launchファイルを使用します。
3つのターミナルを開き、ハードウェア、制御システム、GUIの順に起動してください。

### 0. EtherCATデーモンの準備

EtherCAT構成では、ROSノードを起動する前に`core_hardware_daemon`をsystemdサービスとしてインストールします。
次の操作は、初回セットアップ時と`core_hardware`のビルド成果物を更新したときに実行してください。

```bash
cd ~/core_ws
sudo ./src/core2026/core_hardware/scripts/install_core_hardware_daemon_service.sh
sudo systemctl enable --now core_hardware_daemon
sudo systemctl status core_hardware_daemon
```

インストールスクリプトでは、EtherCAT用ネットワークインターフェースとして`enp2s0`を使用します。
実機のインターフェース名が異なる場合は、スクリプト内の`IF_NAME`を環境に合わせて変更してください。

### 1. ハードウェア

```bash
source ~/core_ws/install/setup.bash
ros2 launch core_launch core_2026_hardware.xml
```

EtherCATデーモンとUNIXドメインソケットで通信するROSハードウェアインターフェースを起動します。

### 2. 制御システム

別のターミナルで実行します。

```bash
source ~/core_ws/install/setup.bash
ros2 launch core_launch core2026_system.xml
```

車体制御、射撃、敵検出、カメラ、モード管理、操縦入力、行動システムを起動します。

### 3. GUI

さらに別のターミナルで実行します。

```bash
source ~/core_ws/install/setup.bash
ros2 launch core_launch core2026_gui.xml
```

操縦者向けQt HUDと、機体状態を表示するステータスGUIを起動します。
