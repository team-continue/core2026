# 腰Yaw・4輪オムニのシステム同定

機体上のROS 2 Humble PCで、腰（ID 4）の速度応答と、足回り（ID 0〜3）の前後・横・旋回応答を記録する。1軸につき推定用と検証用の2本のbagを取り、速度指令から変位までを `K exp(-Ls) / [s (Ts+1)]` で近似する。

| 試験 | 解析の入力 | 出力 |
| --- | --- | --- |
| `waist` | `/hardware/can/tx` のID 4の速度指令（rad/s） | 腰の相対角 `/hardware/joint_states.position[4]`（rad） |
| `vx`, `vy`, `wz` | ID 0〜3の指令から逆算した下回り基準の速度。加速度制限後の値 | 床基準のオドメトリ姿勢から計算した前後・横変位（m）、旋回角（rad） |
| `wheel0`〜`wheel3` | 各輪の速度指令（rad/s） | 同じIDのエンコーダ角（rad）。上の走行bagを再利用できる |
| 上回りの方位保持 | 下回りを旋回させる検証用bag | 上回りIMUの最大方位ずれ・RMS（deg）、停止から許容帯内に戻るまでの時間（s） |

## 起動と事前確認

この手順は `orthros/main` のROS 2 Humbleで検証した。Ubuntu 24.04・ROS 2 Jazzyへ移行した環境では、下の `source` を `source /opt/ros/jazzy/setup.bash` に置き換える。Jazzyでの試験は未実施。

```bash
source /opt/ros/humble/setup.bash
# このブランチを src にチェックアウトしたワークスペースで実行
cd ~/ros2_ws
colcon build --packages-up-to core_tools
source install/setup.bash
ros2 run core_tools base_sysid_probe waist   # 計画の表示のみ。ROSへの接続・指令送信なし
```

- **試験専用の起動に切り替える。** 通常の統合launch・操縦・自動照準・発射は止め、機体は停止・脱力した状態から用意する。通常の制御ノードと試験用制御ノードを二重起動しない。試験launch自体はEtherCAT、停止管理、センサを起動しない。実機には既存のハードウェア・停止管理・上回りIMUを起動し、走行試験ではオドメトリも起動する。
- **停止経路を確かめる。** 前後左右に空間を確保し、物理・遠隔停止、HALT/DOWN、通信途絶で機体側が脱力することを先に確認する。試験プログラムが送る速度ゼロは制御上の停止。競技側の停止が常に優先する。[CoRE-1 §13.7](https://core.scramble-robot.org/rule/core-1-rulenavi/#r-13.7) / [競技システム §10.4](https://core.scramble-robot.org/rule/core-system-rulenavi/#r-10.4)
- **走行時の観測点を合わせる。** 既定は `/localization/odom`（`nav_msgs/Odometry`）。下回り中心の床基準姿勢が入ることを確認する。上回りに付いたLiDARの姿勢を使う場合は先に下回り中心へ座標変換する。フレーム名だけを変えてはいけない。別名の入力はprobeとfit両方に `--odom-topic` と必要に応じて `--base-frame` を渡し、bagの記録対象も変更する。
- 腰試験は下回り停止、走行試験は腰速度指令ゼロで行う。機体正面・エンコーダの符号と減速比を小振幅で確認する。`/control/body_omega` は指令、`/sensing/imu` は回転する上回りの観測値。いずれも下回りの旋回実測値には使わない。

```bash
ros2 launch core_tools base_sysid.launch.py
```

2台の制御ノードだけが `/hardware/can/tx` に出力する構成で使う。試験指令は専用トピックに送り、既存の停止・指令タイムアウトを通す。指令周期は50 Hz。試験中に停止状態・関節・IMU・走行用オドメトリの受信が0.35秒を超えて途絶えた場合や、他のCAN送信ノードを検出した場合は、その回を中断する。走行中の開始位置からの変位0.45 mまたは旋回0.7 rad、腰試験時の変位0.65 radでも中断する。計測停止後は状態と記録を確認してから、新しい回として起動する。

## 1軸の収録

下の例は前後方向。ターミナルAで記録を開始し、トピックが購読されたらターミナルBで試験する。終了後は静止を1秒以上記録してからAをCtrl+Cで閉じる。機体上のPCで収録し、bag名は試行ごとに変える。

```bash
# A
ros2 bag record -o vx_train /sysid/drive /sysid/waist_velocity \
  /hardware/can/tx /hardware/joint_states /control/target_omega /control/body_omega \
  /sensing/imu /localization/odom /system/emergency/hazard_status
# B
ros2 run core_tools base_sysid_probe vx --profile step --amplitude 0.12 --execute
```

次に別bag `vx_valid` を開始して `--profile multistep`（異なる振幅・切替順）で同じ方向を測る。腰は `waist`、横は `vy`、旋回は `wz`。振幅は並進がm/s、腰と旋回がrad/s。初回は小さくする。既定は0.12、許容上限は並進0.20、角速度0.35。腰の位置目標をモータ速度入力に流用しない。試験コマンドの既定は波形表示だけで、送信には `--execute` が必要。

## 解析とUnityとの比較

```bash
ros2 run core_tools base_sysid_fit vx --train vx_train --valid vx_valid --out result_vx
ros2 run core_tools base_sysid_fit wheel0 --train vx_train --valid vx_valid --out result_wheel0
```

- `model.json` に速度ゲイン `gain`、時定数 `tau_s`、むだ時間 `delay_s`、各試行の変位RMSE・fitを出す。`train.csv`、`valid.csv`、`response.png` で実測と予測を比較できる。出力先には新しいディレクトリ名を指定する。
- モデルは静止から開始する単軸近似。指令はホールドして10 ms刻みで計算し、実測の姿勢・関節角と同じ時刻で評価する。bag受信時刻を使うので、同定したむだ時間には通信・推定器の遅れも入る。低速で整数化された速度フィードバックを直接微分・評価する必要はない。
- 長いbagは `--train-window START STOP`、`--valid-window START STOP`（bag先頭からの秒）で静止・励起・停止を含む区間を指定できる。正逆入力、前後の静止指令、連続した計測・停止状態が揃う回だけを解析する。むだ時間が探索上限0.30秒、時定数が0.01秒または2秒付近の場合は波形と時刻を確認する。
- Unityには同じ符号・単位の速度指令を入力し、前後／横の停止距離と腰／車体の旋回角を実機波形に重ねる。`valid.csv` の `speed_command` はセンササンプル時刻での値。厳密なリプレイにはbagの `/hardware/can/tx` の更新時刻を使う。速度制限・飽和は実装されている層で一度だけ適用する。

## 旋回中の方位保持

上の試験launchを終了し、`ros2 launch core_tools base_sysid.launch.py velocity_mode:=false` で起動し直す。起動時の上回り方位を基準に、通常の腰姿勢制御を使ったまま下回りだけに `wz` の波形を送る。この構成の回転モードは通常（0）。競技時の無限回転モードの評価には、通常の操縦経路で収録した専用bagを使う。

```bash
# 上と同じトピックで hold_valid を記録し、別ターミナルで実行
ros2 run core_tools base_sysid_probe wz --profile multistep --amplitude 0.20 --execute
ros2 run core_tools base_sysid_hold hold_valid --out result_hold --band-deg 2
```

`hold.csv` に初期方位からの上回り／下回りの角度、`hold.json` に上回り最大ずれ・RMSと、最終旋回指令がゼロになってから許容帯内に収まるまでの時間を出す。帯内に0.5秒以上留まることを記録内で確認できない場合は `null`。標準試験launchで普段の操縦系は起動しないため、試験後は通常の起動構成に戻す。
