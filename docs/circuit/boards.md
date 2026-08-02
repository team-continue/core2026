# 基板とピン配置

## Teensy 4.1（upper）

`core_hardware/teensy41/upper/src/pin.h` の定義です。全アクチュエータのバスマスタと EtherCAT スレーブを兼ねます。

### EtherCATスレーブコントローラ（AX58100 / SPI1）

| 信号 | ピン | マクロ |
|------|-----|-------|
| MOSI | 26 | `ESC_GPIOX_MOSI` |
| MISO | 39 | `ESC_GPIOX_MISO` |
| SCK | 27 | `ESC_GPIOX_SCK` |
| CS | 38 | `ESC_GPIOX_CS` |
| RSTN | 34 | `ESC_GPIOX_RSTN` |
| IRQ | 37 | `ESC_GPIOX_IRQ` |
| SYNC0 | 6 | `ESC_GPIOX_SYNC0` |
| SYNC1 | 7 | `ESC_GPIOX_SYNC1` |

### その他

| 用途 | ピン / ポート | 設定 | マクロ |
|------|--------------|------|-------|
| 非常停止出力 | 32 | 初期値 HIGH | `PIN_EMERGENCY` |
| 受信機（無線） | Serial5 | 115200 bps、受信バッファ 1024 B | `PORT_WIRELESS` / `LEN_WIRELESS` |
| Feetechサーボ | Serial7 | 1 Mbps | `SERVO_SERIAL` |
| 発射ESC（PWM） | 24 | パルス幅 1000～1400 µs | `PIN_ESC2` |
| LEDテープ（WS2812Serial, GRB） | 24 | 1灯、明度 20 | `LED_UPPER_SERIAL_PIN` |
| ステータスLED | `LED_BUILTIN` | ROS2未接続時に点灯 | — |

!!! danger "ピン24が ESC と LED で重複しています"
    `PIN_ESC2` と `LED_UPPER_SERIAL_PIN` はいずれも **24** に定義されており、`setup()` で `upper_led.init()` と `esc.init()` の両方が実行されます。どちらか一方は意図した動作をしない可能性が高いため、実機の配線と合わせて確認してください（`PIN_ESC1`（35）はコメントアウト済み）。

### 初期化順序

`setup()` は以下の順で初期化します（`upper/src/main.cpp`）。

```
pinMode(PIN_EMERGENCY) → upper_led.init() → ecat_begin()
  → wireless.init() → sts.init() → can3_init() → can2_init() → esc.init()
```

その後 `led_timer` を 50 ms 周期（20 Hz）で起動します。メインループは `upper_led.update()` / `ecat_update()` / `sts.loop()` / `can3_loop()` / `can2_loop()` を回します。

## Teensy 4.1（bottom）

`core_hardware/teensy41/bottom/src/pin.h` の定義です。競技装置とのインタフェースを担当します。

| 用途 | ピン / ポート | 設定 | マクロ |
|------|--------------|------|-------|
| 非常停止出力 | 19 | — | `PIN_EMERGENCY` |
| LEDテープ 1 | 35 | WS2812Serial | `LED1_SERIAL_PIN` |
| LEDテープ 2 | 24 | WS2812Serial | `LED2_SERIAL_PIN` |
| 競技装置シリアル | Serial4 | 115200 bps、受信バッファ 2048 B | `PORT_SERIAL` |
| upperとの通信 | CAN3 | CANフレームID `0xff`（len=2）を受信して3バイト返信 | `can3.h` |

bottom は競技装置からHP・撃破状態・チーム色を受け取り、upper からの問い合わせに応答します。

| 返信バイト | 内容 |
|-----------|------|
| `buf[0]` | destroy（撃破フラグ） |
| `buf[1]` | hp（残HP、初期値 100） |
| `buf[2]` | color（チーム色） |

bottom の `led_timer` も 50 ms 周期で動作し、CAN3 の最終受信から 500 ms 経過すると `LED_BUILTIN` を点灯させます。

## ファームウェアのビルド

PlatformIO を使用します。全プロジェクトともボードは `teensy41`、フレームワークは `arduino` です。

| プロジェクト | 用途 | 主な依存 |
|------------|------|---------|
| `teensy41/upper` | メイン制御基板（EtherCATスレーブ） | SOES、WS2812Serial、FlexCAN_T4 |
| `teensy41/bottom` | 競技装置I/F基板 | FlexCAN_T4 |
| `teensy41/soes_test_ax58100` | AX58100 単体動作確認 | — |
| `teensy41/teesny_u2d2` | U2D2互換 Dynamixel/Feetech ブリッジ（Serial7、送信LED ピン13） | SparkFun ICM-20948 |

`upper` は `build_src_filter` で `src/` に加えて `soes/soes`、`soes/soes-arduino`、`soes/soes-esi/objectlist.c` をビルド対象に含めます。

### EEPROM書き込み

EtherCATスレーブのEEPROMは以下のコマンドで書き込みます。

```bash
sudo ./vendor/soem/bin/eepromtool ifname 1 -w ./teensy41/upper/soes/soes-esi/eeprom.bin
```

### 通信テスト

```bash
cd core_hardware/test
./build.sh
sudo ./build/ecat_zero_check enp2s0
```

## 関連ページ

- [CANバスとモータID](buses.md) — バス構成とID割り当て
- [ハードウェアセットアップ](../guides/hardware-setup.md) — 実機の配線・起動手順
