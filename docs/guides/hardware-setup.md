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
```

## Docker環境での使用

`docker-compose.yaml` で以下の権限が設定済みです:

```yaml
network_mode: host
cap_add:
  - NET_RAW
  - NET_ADMIN
```

## Livox Mid-360 LiDAR

### 接続

- イーサネット接続
- デフォルトIP設定に従いネットワーク設定

### FAST-LIOとの連携

```bash
ros2 launch core_launch navigation.launch.py odom_source:=fastlio
```

FAST-LIO の設定ファイル: `fast_lio/config/mid360.yaml`（fast_lioパッケージ内）
