# 基板とピン配置

## Teensy 4.1（upper）

`core_hardware/teensy41/upper/src/pin.h` と `upper/src/main.cpp` の定義です。EtherCATスレーブとして制御PCから指令を受け、CAN2、CAN3、シリアル、PWMへ振り分けます。

### EtherCATスレーブコントローラ（AX58100 / SPI1）

| 信号 | Teensyピン | マクロ |
|------|-----------|-------|
| MOSI | 26 | `ESC_GPIOX_MOSI` |
| MISO | 39 | `ESC_GPIOX_MISO` |
| SCK | 27 | `ESC_GPIOX_SCK` |
| CS | 38 | `ESC_GPIOX_CS` |
| RSTN | 34 | `ESC_GPIOX_RSTN` |
| IRQ | 37 | `ESC_GPIOX_IRQ` |
| SYNC0 | 6 | `ESC_GPIOX_SYNC0` |
| SYNC1 | 7 | `ESC_GPIOX_SYNC1` |

PDOとEEPROMの設定は [EtherCATとPDO](ethercat.md) を参照してください。

### その他

| 用途 | ピン / ポート | 現在の設定 | マクロ |
|------|--------------|-----------|-------|
| 非常停止出力 | 32 | 初期値 HIGH | `PIN_EMERGENCY` |
| 無線受信機 | Serial5 | 115200 bps、受信バッファ 1024 B | `PORT_WIRELESS` / `LEN_WIRELESS` |
| Feetechサーボ | Serial7 | 1 Mbps、8台 | `SERVO_SERIAL` / `LEN_SERVO` |
| 発射ESC（PWM） | 24 | パルス幅 1000～1400 µs | `PIN_ESC2` |
| LEDテープ（WS2812Serial, GRB） | 35 | 43灯、更新20 ms、明度32 | `LED_UPPER_SERIAL_PIN` |
| ステータスLED | `LED_BUILTIN` | EtherCAT指令またはbottomが未接続のとき点灯 | — |

発射ESCはピン24、upper LEDはピン35であり、現在の `pin.h` では重複していません。`PIN_ESC1` の旧定義はピン35ですがコメントアウトされています。

### 初期化とメインループ

`setup()` は以下の順で初期化します。

```text
pinMode(PIN_EMERGENCY) → upper_led.init() → ecat_begin()
  → wireless.init() → sts.init() → can3_init() → can2_init() → esc.init()
```

その後 `led_timer` を50 ms周期（20 Hz）で起動します。メインループは次の処理を繰り返します。

```text
upper_led.update() → ecat_update() → sts.loop() → esc.loop()
  → can3_loop() → can2_loop()
```

## Teensy 4.1（bottom）

`core_hardware/teensy41/bottom/src/pin.h`、`bottom/src/can3.h`、`bottom/src/main.cpp` の定義です。CAN3上のノードとしてupperから問い合わせを受け、競技装置から受信した状態を返信します。

| 用途 | ピン / ポート | 現在の設定 | マクロ |
|------|--------------|-----------|-------|
| 非常停止出力 | 19 | 出力先として定義 | `PIN_EMERGENCY` |
| LEDテープ（使用中） | 35 | 200灯、更新20 ms、明度32 | `LED1_SERIAL_PIN` |
| LEDテープ2（未使用） | 24 | ピン定義のみ。オブジェクト生成と更新はコメントアウト | `LED2_SERIAL_PIN` |
| 競技装置シリアル | Serial4 | 115200 bps、受信バッファ 2048 B | `PORT_SERIAL` / `MAX_LEN` |
| upperとの通信 | CAN3 | 1 Mbps。標準ID `0xff`（2 B）を受信し、標準ID `0x00`（3 B）で返信 | `can3.h` |

現在動作するbottom LEDはピン35の1本です。CAN受信データの先頭バイトをLEDモードとして使用し、2バイト目はファームウェア上で使用していません。

bottomは競技装置からHP、撃破状態、チーム色を受け取り、upperからの問い合わせに次の3バイトで応答します。

| 返信バイト | 内容 |
|-----------|------|
| `buf[0]` | destroy（撃破フラグ） |
| `buf[1]` | hp（残HP、初期値100） |
| `buf[2]` | color（チーム色） |

bottomの `led_timer` も50 ms周期で動作します。upperからのCAN3問い合わせが500 ms以上途絶えると `LED_BUILTIN` を点灯します。

## ファームウェアのビルド

PlatformIOを使用します。各プロジェクトのボードは `teensy41`、フレームワークはArduinoです。

| プロジェクト | 用途 | 主な依存・構成 |
|------------|------|---------------|
| `teensy41/upper` | メイン制御基板、EtherCATスレーブ | SOES、WS2812Serial、FlexCAN_T4、Servo |
| `teensy41/bottom` | 競技装置I/F、CAN3ノード | WS2812Serial、FlexCAN_T4 |
| `teensy41/soes_test_ax58100` | AX58100単体動作確認 | SOES |
| `teensy41/teesny_u2d2` | U2D2互換 Dynamixel / Feetechブリッジ | Serial7、送信LED ピン13、SparkFun ICM-20948 |

upperは `build_src_filter` で `src/` に加えて `soes/soes`、`soes/soes-arduino`、`soes/soes-esi/objectlist.c` をビルド対象に含めます。

```bash
cd core_hardware/teensy41/upper
pio run

cd ../bottom
pio run
```

## 関連ページ

- [CANバスとモータID](buses.md) — CAN2 / CAN3の構成とID割り当て
- [EtherCATとPDO](ethercat.md) — AX58100、PDO、EEPROM書き込み、通信テスト
