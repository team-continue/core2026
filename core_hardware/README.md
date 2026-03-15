# core_hardware

## core_hardware_daemon を systemd で起動する

### 1. build

```bash
cd ~/core_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select core_hardware --symlink-install
```

### 2. service をインストール

```bash
cd ~/core_ws
sudo ./src/core2026/core_hardware/scripts/install_core_hardware_daemon_service.sh
```

このコマンドで以下が配置されます．

- `/etc/systemd/system/core_hardware_daemon.service`
- `/opt/core_hardware/bin/core_hardware_daemon`

`core_hardware_daemon.service` は `/opt/core_hardware/bin/core_hardware_daemon` を絶対パスで直接実行します．
現在の固定引数は以下です．

- `--if-name enp2s0`
- `--socket-path /tmp/core_hardware.sock`

`--socket-path` は `launch/core_hardware.launch.py` の `socket_path` と同じ値にしてください．

### 3. 自動起動を有効化して起動

```bash
sudo systemctl enable --now core_hardware_daemon
```

### 4. 状態確認

```bash
sudo systemctl status core_hardware_daemon
sudo journalctl -u core_hardware_daemon -f
```

### 5. build 成果物を更新したとき

`colcon build --packages-select core_hardware` を再実行したあとで，再度以下を実行します．

```bash
sudo ./src/core2026/core_hardware/scripts/install_core_hardware_daemon_service.sh
```

既に service file がある場合は `/opt/core_hardware/bin/core_hardware_daemon` だけが更新されます．
そのあと `core_hardware_daemon` は自動で restart されます．

### 6. service を削除するとき

```bash
sudo ./src/core2026/core_hardware/scripts/remove_core_hardware_daemon_service.sh
```

このコマンドで service を stop / disable したうえで，以下を削除します．

- `/etc/systemd/system/core_hardware_daemon.service`
- `/opt/core_hardware/bin/core_hardware_daemon`

## eepromへの書き込み方

```
sudo ./vendor/soem/bin/eepromtool ifname 1 -w ./teensy41/upper/soes/soes-esi/eeprom.bin
```

## testコード

build
```
cd test
./build.sh
```

exec
```
sudo ./build/ecat_zero_check enp2s0
```

## 現在の EtherCAT 構成に関する注意

現在は実機で安定して動作している PDO 構成をそのまま採用しています．

- `motor_state_pos` は 15 軸すべて publish します
- `id 0..3` は state frame の `position` / `velocity` は通常どおり使いますが，`torque` は使いません
- `wireless` 7 byte は EtherCAT PDO 上では独立した field を持たず，`motor_state_torque[0..3]` の先頭 7 byte に載せています
- この多重化は `core_hardware_daemon` と Teensy 間で吸収し，ROS2 側の `wireless` topic インタフェースは維持します

そのため EtherCAT レイアウトを変更するときは，`utypes.h` だけでなく `objectlist.c` / `esi.json` / PC 側 `ecat.cpp` を必ず同時に揃えてください．
