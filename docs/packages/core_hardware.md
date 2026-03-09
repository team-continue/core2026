# core_hardware

EtherCAT（SOEM）によるハードウェアインターフェースパッケージです。

## 概要

- SOEM ライブラリを使用したEtherCAT通信
- Teensy41スレーブコントローラとの通信
- モータESC制御

## 構成

```
core_hardware_node ──EtherCAT──▶ Teensy41スレーブ ──▶ モータESC
```

## EEPROMへの書き込み

Teensy41スレーブのEEPROM設定:

```bash
sudo ./vendor/soem/bin/eepromtool ifname 1 -w ./teensy41/upper/soes/soes-esi/eeprom.bin
```

## テスト

```bash
# ビルド
cd core_hardware/test
./build.sh

# 実行（root権限が必要）
sudo ./build/ecat_zero_check ifname
```

## 起動

```bash
ros2 launch core_hardware core_hardware.launch.py
```

!!! warning "権限"
    EtherCAT通信には `NET_RAW` / `NET_ADMIN` 権限が必要です。Docker環境では `docker-compose.yaml` で設定済みです。
