# ハードウェアセットアップ

## システム構成

```mermaid
graph LR
    ROS2["ROS2<br/>core_hardware"] -->|EtherCAT| Teensy["Teensy41<br/>スレーブ"]
    Teensy --> ESC["モータESC"]
    ESC --> Motor["モータ"]
```

## EtherCAT（SOEM）

### 前提条件

- SOEM ライブラリ（`core_hardware/vendor/soem/` に同梱）
- root 権限または `NET_RAW` / `NET_ADMIN` ケーパビリティ

### ネットワークインターフェース

EtherCATで使用するネットワークインターフェース名を確認:

```bash
ip link show
```

### Teensy41 EEPROM書き込み

```bash
cd core_hardware
sudo ./vendor/soem/bin/eepromtool <ifname> 1 -w ./teensy41/upper/soes/soes-esi/eeprom.bin
```

`<ifname>` は EtherCAT 用のネットワークインターフェース名に置き換えてください。

## 起動

### ハードウェアインターフェース

```bash
ros2 launch core_hardware core_hardware.launch.py
```

### ボディコントローラ

```bash
ros2 launch core_body_controller body_controller.launch.py
```

## 動作確認

### EtherCAT通信テスト

```bash
cd core_hardware/test
./build.sh
sudo ./build/ecat_zero_check <ifname>
```

### トピック確認

```bash
# CAN指令の確認
ros2 topic echo /can/tx

# ジョイントステートの確認
ros2 topic echo /joint_states

# DM-IMU-L1の姿勢と発行周期
ros2 topic echo /imu --once
ros2 topic hz /imu
```

## Docker環境での使用

`docker-compose.yaml` で以下の権限が設定済みです:

```yaml
network_mode: host
cap_add:
  - NET_RAW
  - NET_ADMIN
```

## Damiao DM-IMU-L1

### 取り付けとUSB接続

DM-IMU-L1を無限回転する上部車体に剛体固定し、センサの+Xを機体前方、+Zを上向きにします。
Type-CケーブルでPCへ直結し、デバイス名を確認します。

```bash
ls -l /dev/serial/by-id/
ls -l /dev/ttyACM*
```

既定値は本番機の
`/dev/serial/by-id/usb-DM-IMU_DM-IMU_USB_CDC_2025021200-if00`です。別のIMUを使う場合は
`imu_port:=/dev/serial/by-id/...`で上書きしてください。

デバイスを一般ユーザーで開けない場合は、ユーザーを`dialout`グループへ追加して再ログインします。

```bash
sudo usermod -aG dialout $USER
```

### 事前ジャイロ校正

公式[DM-IMUリポジトリ](https://github.com/dmBots/DM-IMU)のWindows用`DM-IMU-Upper`を使用します。
IMUを実機へ固定して完全に静止させ、公式V1.2マニュアルの手順でジャイロ校正を実行します。
校正処理が完了するまで機体へ振動を与えず、必要な保存操作はこの事前作業でのみ行います。

ROSドライバは起動のたびに加速度・角速度・Euler出力と200 Hzを揮発設定しますが、自動校正や
フラッシュ保存は行いません。

### 単体確認

```bash
ros2 launch core_damiao_imu damiao_imu.launch.py
ros2 topic hz /imu
ros2 topic echo /imu --once
```

上から見て機体を反時計回りに回したときYawが正になることを確認してください。逆になる場合は
ソフトウェアで符号反転せず、まず+X前方・+Z上向きの取り付け方向を修正します。

## Livox Mid-360 LiDAR

### 接続

- イーサネット接続
- デフォルトIP設定に従いネットワーク設定

### FAST-LIOとの連携

```bash
ros2 launch core_launch navigation.launch.py odom_source:=fastlio
```

FAST-LIO の設定ファイル: `fast_lio/config/mid360.yaml`（fast_lioパッケージ内）
