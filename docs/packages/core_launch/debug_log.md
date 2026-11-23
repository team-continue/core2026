# debug_log

## Purpose

「なんでこれ動かんかったんだろう」を後から調べられるよう、機体の PC に要点のトピックと `/rosout`（全ノードのログ）を `ros2 bag` で残します。今の値はスマホのアプリでも見られますが、過去の値はここでしか見返せません。

## 起動

`core2026_system.xml` を起動すると、いつも一緒に記録します。止めるときは `debug_log:=false` を付けます。

```bash
ros2 launch core_launch core2026_system.xml debug_log:=false
```

記録だけを起動することもできます。

```bash
ros2 launch core_launch debug_log.launch.py
```

| 引数 | 既定値 | 意味 |
|---|---|---|
| `debug_log_dir` | `$CORE_DEBUG_LOG_DIR`、無ければ `~/rosbag/core_debug_log` | 記録を置くディレクトリ |
| `debug_log_config` | `core_launch/config/debug_log.yaml` | 記録するトピックと上限 |

docker（`docker-compose.yaml`）では `CORE_DEBUG_LOG_DIR=/rosbag/core_debug_log` なので、ホストの `~/rosbag/core_debug_log` に残ります。

## 記録するもの

名前が `include` のどれかに当てはまり、`exclude` のどれにも当てはまらないトピックを記録します（どちらも部分一致の正規表現）。あとから起動したノードのトピックも拾います。

- **拾う:** `/rosout`、`/tf_static`、`/system/`（緊急停止の要因・health）、`/ui/`（操縦の入力）、`/bridge/`、`/behavior/`、`/control/`（cmd_vel など）、`/hardware/`（CAN・joint_states・モータの状態・無線の受信）、`/mecha/`（射撃）、`/planning/`、`/localization/`、敵検出の `target_pose`
- **除く:** 画像・点群・コストマップ（名前に `image`・`compressed`・`cloud`・`costmap`・`global_map` などを含むもの）、`/tf`、`/imu`

## 置き場所と容量

起動のたびに、記録を置くディレクトリの下に `0001_20261124-005113` のような**連番＋時刻**のディレクトリを作ります。機体の時計がずれても、連番で順番が分かります。

| 設定 (`debug_log.yaml`) | 既定値 | 意味 |
|---|---|---|
| `max_total_mb` | 4000 | 合計の上限。起動のたびに、超えた分を古い記録から消す |
| `min_free_mb` | 2000 | ディスクの空きの下限。足りなければ古い記録から消す。全部消しても足りないときは、何も消さずに記録しない（機体の起動は止めない） |
| `split_duration_s` | 300 | 1 ファイルの長さ [s] |
| `max_cache_mb` | 1 | 書き込む前にためておく量。急に電源を切ったときに失うのはこの分まで |

- 消すのは連番＋時刻の名前のディレクトリだけです。手で置いた bag は消しません
- 上限は起動のときにだけ見ます。起動したあとの記録はそのぶん超えます
- 手元で実機に近い量（約 2,000 メッセージ/秒）を流すと、**1 分で約 17 MB（1 時間で約 1 GB）**、記録の CPU は 1 コアの約 15%、メモリは約 70 MB でした

## 見返し方

`debug_log_timeline.py` で、`/rosout` の WARN 以上と、主な状態（緊急停止の要因・health・bridge・モード・behavior の状態・HP）が**変わった時**の値を時刻順に出します。bag を指定しなければ、いちばん新しい記録を読みます。

```bash
ros2 run core_launch debug_log_timeline.py
```

```text
# /rosbag/core_debug_log/0012_20261124-005113  (start 2026-11-24 00:51:13)
00:51:13.207    +0.023s  /system/emergency/hazard_label = "NONE"
00:51:14.810    +1.626s  /system/health/diagnostics motor/wheel = OK
00:52:40.002   +86.818s  [WARN] hardware.core_hardware: ...
00:52:40.015   +86.831s  /system/emergency/hazard_label = "BRIDGE DISCONNECTED"
```

| オプション | 意味 |
|---|---|
| `--list` | 記録の一覧と大きさ |
| `--level info` | `/rosout` を INFO 以上まで出す（既定は `warn`） |
| `--watch REGEX` | 変わった時に値を出すトピックを足す（例: `--watch '^/control/cmd_vel$'`） |
| `<bag>` | 読む記録のディレクトリ |

値をグラフで見るときや再生するときは、ふつうの bag として扱えます（`ros2 bag info`・`ros2 bag play`・PlotJuggler など）。

## 急に電源を切ったとき

sqlite3 の `resilient` プリセット（WAL）で書いているので、急に電源を切っても直前まで読めます（手元で `kill -9` したときは、切る約 0.2 秒前まで読めました）。ただし `metadata.yaml` が書かれないため、`ros2 bag info` などは開けません。`debug_log_timeline.py` は自動で作り直します。手で作り直すときは次のとおりです。

```bash
ros2 bag reindex <記録のディレクトリ> -s sqlite3
```
