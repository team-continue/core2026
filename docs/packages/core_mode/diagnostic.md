# diagnostic

## Purpose

マイコンやワイヤレス受信機との通信が途絶えても、それ自体は「メッセージが来ない」という受動的な事象でしかなく、緊急停止のトリガーにはなりません。このノードは各系統のハートビートを監視し、一定時間途絶えたことを能動的な緊急信号に変換します。

## Inner-workings / Algorithms

`diagnostic_cycle`（デフォルト50ms）の周期でタイマーを回し、各監視対象の最終受信時刻からの経過時間をチェックします。

| 監視対象 | 入力 | タイムアウト | 出力 |
|---------|------|------------|------|
| マイコン | `microcontroller_monitor`（`/joint_states`） | `microcontroller_diagnostic_time` | `microcontroller_emergency` |
| ワイヤレス受信機 | `receive_module_monitor`（`/wireless`） | `receiver_diagnostic_time` | `receiver_emergency` |

ハートビート専用のトピックを新設せず、常時流れている既存のトピック（`/joint_states` と `/wireless`）を生存確認に流用しています。これにより、通信経路そのものが生きていることを直接確認できます。

経過時間がタイムアウトを超えた場合に `true`、受信が回復すれば `false` を発行し、[emergency_handler](emergency_handler.md) が他の緊急源とまとめて最終判定を行います。

## Inputs / Outputs

### Input

| トピック | 型 | 説明 |
|---------|------|------|
| `microcontroller_monitor` | `sensor_msgs/JointState` | マイコンハートビート（launchで `/joint_states` にリマップ） |
| `receive_module_monitor` | `std_msgs/UInt8MultiArray` | 受信機ハートビート（launchで `/wireless` にリマップ） |

### Output

| トピック | 型 | 説明 |
|---------|------|------|
| `microcontroller_emergency` | `std_msgs/Bool` | マイコン通信途絶フラグ |
| `receiver_emergency` | `std_msgs/Bool` | 受信機通信途絶フラグ |

## Parameters

設定ファイル: `config/mode.params.yaml`

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `microcontroller_diagnostic_time` | int | `2000` | マイコンハートビートタイムアウト [ms] |
| `receiver_diagnostic_time` | int | `2000` | 受信機ハートビートタイムアウト [ms] |
| `diagnostic_cycle` | int | `50` | 診断チェック間隔 [ms] |

## Assumptions / Known limits

- 監視するのはメッセージの到着有無のみで、内容の妥当性は検証しません。マイコンが不正な値を送り続けている場合は正常と判定されます。
- タイムアウト値（デフォルト2秒）は検知の遅れに直結します。短くすると通信の一時的なゆらぎで誤検知しやすくなるため、実機の通信品質に合わせた調整が必要です。
- 起動直後は一度もメッセージを受信していない状態です。初回受信までの扱いはタイムアウト設定に依存するため、全ノードの起動順序によっては起動直後に緊急信号が出ることがあります。
