# ノードドキュメントの書き方

各ROS2ノードのドキュメントは、[Autoware Universe](https://autowarefoundation.github.io/autoware_universe/main/)の構成に倣います。パッケージが複数ノードを持つ場合は、**ノードごとに個別ページ**を作成してください。

## ディレクトリ構成

```
docs/packages/
└── <package_name>/
    ├── index.md              # パッケージ概要（ノード一覧・データフロー図など）
    ├── <node_name_1>.md      # ノード1個別ページ
    └── <node_name_2>.md      # ノード2個別ページ
```

ノードが1つしかないパッケージは `docs/packages/<package_name>.md` のように単一ファイルのままで構いません。

`mkdocs.yml` の `nav` には次のようにネストして追加します:

```yaml
nav:
  - パッケージ:
    - core_example:
      - 概要: packages/core_example/index.md
      - node_a: packages/core_example/node_a.md
      - node_b: packages/core_example/node_b.md
```

## パッケージ概要ページ（index.md）のテンプレート

```markdown
# <package_name>

<パッケージが何をするものかを1〜2文で>

## ノード一覧

| ノード | 役割 |
|-------|------|
| `<node_a>` | <1行説明> |
| `<node_b>` | <1行説明> |

## データフロー

\`\`\`
<上流ノード> ──▶ <このパッケージのノード群> ──▶ <下流ノード>
\`\`\`

## 起動方法

\`\`\`bash
ros2 launch <package_name> <launch_file>.launch.py
\`\`\`
```

## ノード個別ページのテンプレート

見出しの順序は固定です。該当内容がない節は省略せず「なし」と明記するか、節ごと削除してください（Parametersなど空になりえない節を除く）。

```markdown
# <node_name>

## Purpose

<このノードが何のために存在するかを1〜2文で。「何をするか」ではなく「なぜ必要か」を書く>

## Inner-workings / Algorithms

<処理の流れ、使用しているアルゴリズム、状態遷移など。図が有効な場合はmermaidまたはASCII図を挿入>

\`\`\`mermaid
graph LR
  A[入力トピック] --> B[処理]
  B --> C[出力トピック]
\`\`\`

## Inputs / Outputs

### Input

| トピック / サービス | 型 | QoS | 説明 |
|-------------------|-----|-----|------|
| `/example/input` | `example_msgs/Example` | reliable(10) | <説明> |

### Output

| トピック / サービス | 型 | QoS | 説明 |
|-------------------|-----|-----|------|
| `/example/output` | `example_msgs/Example` | reliable(10) | <説明> |

## Parameters

設定ファイル: `param/<file>.yaml`（該当する場合）

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `param_name` | double | `1.0` | <説明・単位> |

## Assumptions / Known limits

<このノードが前提としている条件（例: 特定のフレームが存在する、特定の順序でメッセージが届く）と、既知の制限事項・未対応ケース。特になければ「特になし」と記載>
```

## 各節の書き方のポイント

| 節 | 目的 |
|----|------|
| Purpose | 「何をするか」ではなく「なぜあるか」を1〜2文で。詳細は次節以降に譲る |
| Inner-workings / Algorithms | 実装を読まなくても処理の流れが追えるように。座標フレームや状態遷移など、コードだけでは読み取りにくい設計判断を優先的に書く |
| Inputs / Outputs | サブスクライブ/パブリッシュするトピックだけでなく、使用するサービス・アクション・パラメータサーバも含める |
| Parameters | `param/*.yaml` や launch のデフォルト引数と一致させる。単位は必ず併記する（例: `[m/s]`、`[Hz]`） |
| Assumptions / Known limits | レビューやデバッグで踏み抜きやすい前提条件を書く。「〇〇が起動していないと動かない」「マルチスレッドを想定していない」など |

## 既存パッケージへの適用

既存の `docs/packages/*.md`（単一ファイル形式）は、本テンプレートへの移行を必須とはしません。新規パッケージ・新規ノードのドキュメントを作成する際にこの形式を使用してください。既存ドキュメントを移行する場合は、内容を書き直さずディレクトリ構成と見出しのみを合わせることを優先してください。
