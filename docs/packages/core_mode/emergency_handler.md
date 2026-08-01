# emergency_handler

## Purpose

緊急停止の要因は物理スイッチ、ソフトウェア操作、通信途絶、破壊判定と複数あり、それぞれ別のトピックで届きます。各制御ノードがこれらを個別に購読すると条件が分散して抜け漏れが生じるため、このノードが全ての緊急源を集約し、単一の `hazard_status` として提供します。

## Inner-workings / Algorithms

各緊急源のフラグを内部状態として保持し、いずれか1つでも成立していれば `hazard_status` を `true` にします（論理和）。

集約する緊急源は以下の5つです。

| 緊急源 | 入力トピック | 発生元 |
|-------|------------|-------|
| ハードウェアスイッチ | `emergency_switch` | 物理緊急停止スイッチ |
| ソフトウェア | `software_emergency` | 操縦UI・上位ソフトからの緊急指示 |
| マイコン通信途絶 | `microcontroller_emergency` | [diagnostic](diagnostic.md) |
| 受信機通信途絶 | `receiver_emergency` | [diagnostic](diagnostic.md) |
| 破壊判定 | `destroy` | ハードウェアからの破壊通知 |

状態が変化するたびに3つのトピックを発行します。

- `hazard_status`: 総合判定（Bool）。各制御ノードが停止条件として参照する
- `hazard_states`: 個別状態の配列。どの要因で停止しているかをデバッグ・GUI表示するために使う
- `hazard_label`: 人間可読なラベル文字列

出力は状態が確実に届くようQoS設定を行っており、後から起動したノードも現在のハザード状態を取得できます。

## Inputs / Outputs

### Input

| トピック | 型 | 説明 |
|---------|------|------|
| `emergency_switch` | `std_msgs/Bool` | ハードウェア緊急スイッチ（launchで `/emergency` にリマップ） |
| `software_emergency` | `std_msgs/Bool` | ソフトウェア緊急指示（launchで `/software_emergency` にリマップ） |
| `destroy` | `std_msgs/Bool` | 破壊/中止コマンド（launchで `/destroy` にリマップ） |
| `microcontroller_emergency` | `std_msgs/Bool` | マイコン通信途絶（`diagnostic` から） |
| `receiver_emergency` | `std_msgs/Bool` | 受信機通信途絶（`diagnostic` から） |

### Output

| トピック | 型 | 説明 |
|---------|------|------|
| `hazard_status` | `std_msgs/Bool` | 総合ハザード状態（true=緊急停止中） |
| `hazard_states` | `std_msgs/Int8MultiArray` | 個別ハザード状態 [switch, software, micro, receiver, destroy] |
| `hazard_label` | `std_msgs/String` | 人間可読なハザードラベル |

## Parameters

設定ファイル: `config/mode.params.yaml`

このノード固有のパラメータはありません。タイムアウト関連のパラメータは [diagnostic](diagnostic.md) 側で定義されています。

## Assumptions / Known limits

- 緊急状態の解除は、全ての緊急源がクリアされることで自動的に行われます。オペレータによる明示的なリセット操作は必要としません。裏を返せば、原因が一時的に解消しただけでも走行が再開する可能性があります。
- このノード自体がクラッシュした場合、`hazard_status` は発行されなくなります。購読側は「hazard_statusが来ないこと」を安全側に倒す扱いにはしていないため、ソフトウェアのみに依存しない物理的な緊急停止手段が別途必要です。
- 旧実装にあった `emergency_button_on` / `emergency_button_off` の2トピック構成は、現在は単一の `software_emergency` に統合されています（該当コードはコメントアウト済み）。
