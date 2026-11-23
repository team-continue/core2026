# 実機で地図を作って自律移動する

研究室や廊下など、CoRE のフィールドではない場所で、実機の Mid-360 と FAST-LIO で地図を作り、その地図で自律移動を起動する手順です。

```
1. 地図を作る      mapping.launch.py（FAST-LIO）→ PCD を保存
2. 地図をそろえる  make_map_files.py → 占有地図・フィールドの範囲
3. 自律移動        core2026_system.xml map_name:=<地図の名前>
```

!!! note "確かめたこと（2026-11-23、sim）"
    - Gazebo の CoRE-1 のフィールドで、走りを rosbag に録って `mapping.launch.py` に流し、地図を作りました。
      作った地図で、7 つのゴールを回るシナリオを走らせました。

        | 地図の作り方 | 走らせた回数 | ゴール | 構造物までの最小距離 | 構造物に触れた回数 | 自己位置のずれ（95%） |
        |---|---|---|---|---|---|
        | **ゆっくり（平均 0.5m/s、既定の設定）** | 3 | すべて 7/7 | 0.17〜0.26m | 0 | 0.01m |
        | 速く（最高 3m/s） | 6 | すべて 7/7 | -0.05〜0.21m | 3 回の走りで触れた | 0.02m |
        | 正しい地図（参考） | 4 | すべて 7/7 | 0.17〜0.36m | 0 | 0.01m |
        | ゆっくり、`real_cautious` | 3 | すべて 7/7 | 0.64〜0.65m | 0 | 0.01m |
        | 速く、`real_cautious` | 3 | すべて 7/7 | 0.69〜0.70m | 0 | 0.02m |

    - ゆっくり作った地図は、正しい地図とのずれの偏りが 2mm 以下、床の高さ -0.01m、床の傾き 0.05° でした。
    - 速く走って作ると、薄い台や裏側の面の点が欠け、計画が構造物に近い道を選びます（自己位置のずれは小さいまま）。
      **ゆっくり、構造物の周りを回って作ってください**。ボクセルを 0.05m に細かくしても良くはなりませんでした。
    - 安全側の設定（`planning_params_file:=real_cautious`、3 章）では、構造物まで 0.64m 以上離れて走りました。
      7 つのゴールを回る時間は約 55 秒から約 117 秒に延びます。
    - 実機と Humble では、まだ試していません。下の「実機で要確認」を見てください。

## 0. 準備

- 機体の PC は Ubuntu 22.04 + ROS 2 Humble です。
- Livox のドライバ本体（Livox-SDK2 と livox_ros_driver2）は、各自で入れます。core2026 の依存には書いていません。
    - `livox_ros_driver2` の `config/MID360_config.json` の IP（機体の PC と Mid-360）が、機体のネットワークに合っていること。
- core2026 は `feature/single-lidar-nav` を使います。

```bash
cd ~/core_ws/src/core2026
```

```bash
git fetch origin
```

```bash
git switch feature/single-lidar-nav
```

```bash
git pull --ff-only
```

外部パッケージ（FAST_LIO・lidar_localization_ros2・ndt_omp_ros2）を `src/thirdparty` に取り、パッチを当てます（何度実行してもよい）。

```bash
bash ~/core_ws/src/core2026/core_launch/scripts/setup_thirdparty.sh
```

### ビルド

core2026 のパッケージは livox_ros_driver2 を依存に書いていないので、**先に livox だけ**ビルドして読み込みます。
`-DCMAKE_BUILD_TYPE=Release` は必ず付けます（付けないと最適化なしになり、自己位置が遅れます）。

```bash
cd ~/core_ws
```

```bash
source /opt/ros/humble/setup.bash
```

```bash
rosdep install --from-paths src -i -y
```

```bash
colcon build --symlink-install --packages-select livox_ros_driver2 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

```bash
source install/setup.bash
```

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

```bash
source install/setup.bash
```

!!! warning "Humble でのビルド"
    `feature/single-lidar-nav` は Jazzy でしか試していません。core2026 の CI（Humble）も、Livox を使うパッケージ
    （core_launch・core_localization・core_perception）はビルドしていません。ビルドで止まったら、エラーの出た
    パッケージと最初のエラーを控えてください。

## 1. 地図を作る

### 1-1. 置き場所を決める

- 作り始めの機体の位置と向きが、地図の原点（x=0、y=0、向き 0 = 機体の前が +x）になります。
  自律移動を起動するときも、**同じ位置と向き**に置きます。床にテープで位置と向きの印を付けておきます。
- **水平な床**に置きます。作り始めの姿勢で、地図の上下（床が z=0）が決まります。
- 人や動く物の少ないときに作ります（少しの間だけ見えた点は除きますが、止まっている人は地図に残ります）。

### 1-2. 録画と地図作りを起動する

ターミナルを 2 つ開きます。どちらも `source ~/core_ws/install/setup.bash` してから実行します。
`core2026_system.xml` や `navigation.launch.py` は**起動しません**（FAST-LIO が 2 つになります）。

ターミナル 1: 地図を作る（Livox のドライバ、URDF の固定 TF、FAST-LIO、地図の保存のノードを起動）

```bash
ros2 launch core_launch mapping.launch.py output:=$HOME/maps/lab.pcd
```

ターミナル 2: 生のデータを rosbag に録る（地図がうまくできなかったとき、後で作り直せます）

```bash
ros2 run core_launch record_localization_bag.sh $HOME/bags/lab_$(date +%Y%m%d_%H%M)
```

### 1-3. 機体を動かす

- 起動してから **5 秒ほど止めたまま**にします（FAST-LIO が IMU の初期値を決めます）。
- 手で押すのがいちばん簡単です。手動操縦で動かすときは、ハードウェアと車体制御と操縦アプリのブリッジだけを
  起動します（`core2026_system.xml` は自律移動も起動するので使いません。この組み合わせは実機で要確認）。
- **ゆっくり**（0.5m/s 以下）動かし、その場で速く回りません。速く走ると、薄い物や裏側の面の点が地図から欠け、
  自律移動で物に近づきすぎます（sim で確かめました）。
- 机・棚・柱など、走らせたい所の近くの物は、**周りを回って**いろいろな向きから見せます。
- 走らせたい範囲を一通り回り、**作り始めの位置に戻って**きます。
- 長い周回（建物を一周するなど）では FAST-LIO のずれがたまり、壁が二重になります。そのときは、録った rosbag から
  GLIM などで作り直すことを考えます。

RViz で、貯まっていく地図を確かめられます。

```bash
rviz2
```

- Fixed Frame: `odom`
- Add → By topic → `/mapping/map_saver/map`（PointCloud2）

壁が二重になっていないか、戻ってきた所の壁が重なっているかを見ます。

### 1-4. 保存する

```bash
ros2 service call /mapping/map_saver/save std_srvs/srv/Trigger
```

`… 点を保存しました … 床の高さの目安 -0.01m（0 に近いはず）、床の傾き 0.05°（0.3° 以下のはず）` のように出れば成功です。
- 床の高さが ±0.05m より大きくずれていれば、URDF の LiDAR の取り付けの値が違うおそれがあります。
- 床の傾きが 0.3° より大きければ、作り始めに機体が水平でありませんでした。遠くの床が静止物（壁）として
  扱われるおそれがあるので、水平な所から作り直します。

保存したら、ターミナル 1 と 2 を Ctrl-C で止めます（止めたときにも、もう一度保存します）。

## 2. 地図のファイルをそろえる

保存した PCD から、UI に出す占有地図とフィールドの範囲を作り、`core_launch/maps` に置きます。

```bash
ros2 run core_launch make_map_files.py $HOME/maps/lab.pcd --name lab --maps ~/core_ws/src/core2026/core_launch/maps
```

`core_launch/maps` に `lab.pcd`・`lab.png`・`lab.yaml`・`lab_field.yaml` ができます。

- `lab.png`: 黒が壁など（床から 0.15〜1.5m に点のある所）、白が走って見た床、灰色が見ていない所。
- `lab_field.yaml`: フィールドの範囲（白の外接の長方形）。計画がゴールや経路を置ける範囲です。
  窓の外の床などが入って広すぎるときは、`--field X_MIN X_MAX Y_MIN Y_MAX` で決めて作り直します（`--force` で書き換え）。

作ったファイルを install に入れるため、core_launch をビルドし直します。

```bash
cd ~/core_ws
```

```bash
colcon build --symlink-install --packages-select core_launch --cmake-args -DCMAKE_BUILD_TYPE=Release
```

```bash
source install/setup.bash
```

!!! note "地図のファイルを git に入れるか"
    `*.pcd` は .gitignore に入っているので commit されません。`lab.png`・`lab.yaml`・`lab_field.yaml` は
    git の管理外のファイルとして残ります。ほかの PC でも使うなら、PCD は別の方法で渡します。

## 3. 自律移動を起動する

1. 機体を、地図を作り始めた**印の位置と向き**に置きます。
2. いつもの手順で、ハードウェアを起動します（[クイックスタート](quick-start.md)）。
3. 全体を、作った地図の名前で起動します。

```bash
ros2 launch core_launch core2026_system.xml map_name:=lab planning_params_file:=real_cautious
```

`planning_params_file:=real_cautious` は、壁や物との間隔を安全側に倒す設定です（`core_launch/config/real_cautious.yaml`）。

- 機体と静止物の間の余裕を 0.1m → 0.25m にします（大域計画の膨らみ、局所計画、安全停止）。
- 速さの上限を 3.0m/s → 1.0m/s にします。
- 余裕を広げるので、狭い所は通れなくなります（既定でも幅 1.6m 以下の長い通路は通れません。この設定では、
  さらに約 0.3m 広い所が要ります）。通れない所にゴールを置くと、手前で止まるか、計画に失敗します。
- 走りに慣れたら、`planning_params_file:=` を外すと既定の設定に戻ります。

自律移動の部分だけを起動するときは、次のようにします。

```bash
ros2 launch core_launch navigation.launch.py environment:=real map_name:=lab planning_params_file:=real_cautious
```

印と違う所から始めるときは、`init_x:=`・`init_y:=`・`init_yaw:=`（地図の上の位置 [m] と向き [rad]）を付けるか、
RViz の「2D Pose Estimate」で位置を合わせます。

### 3-1. 自己位置を確かめる

- RViz で、点群が地図の壁に重なっていること。
- 自己位置の状態が正常であること。

```bash
ros2 topic echo --once /localization/health
```

### 3-2. ゴールを出す

操縦アプリの地図（`/map`。`lab.png` が出ます）でゴールを置きます。アプリを使わないときは、次で送れます
（地図の座標。x・y は m、向きは四元数）。

```bash
ros2 topic pub --once /ui/auto/start geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}"
```

止めるとき:

```bash
ros2 topic pub --once /ui/auto/stop std_msgs/msg/Bool "{data: true}"
```

!!! warning "初めて走らせるとき"
    `planning_params_file:=real_cautious` を付けて起動します。非常停止をすぐ押せる所に人を置き、1〜2m 先の
    近いゴールから始めます。速さの上限を局所計画の速さ（`mppi.max_speed_mps`）より下げたいときは、操縦アプリの
    速度の上限（`/ui/auto/max_speed`）で下げます。速さを下げても壁との間隔は広がりません（局所計画の静止物の余裕は
    「余裕 + 速さ × 0.1 秒」なので、遅いほど近くを通ります）。

## うまくいかないとき

| 症状 | 原因と対処 |
|---|---|
| `output:=<保存する PCD の場所> を指定してください` | `mapping.launch.py` に `output:=` を付ける |
| `… がすでにあります` | 別の名前にするか、`overwrite:=true` を付ける |
| 地図が貯まらない（`/mapping/map_saver/map` が空） | `ros2 topic hz /sensing/livox/lidar`（約 10Hz）と `/sensing/livox/imu`（約 200Hz）を確かめる。届かなければ Livox のドライバの設定（IP）を見る |
| `床が z=0 にありません` | 作り始めに機体が水平でなかったか、取り付けの値が違う。水平な所から作り直す |
| `Unknown map_name 'lab'` | 足りないファイルが表示される。`make_map_files.py` の後に core_launch をビルドし直したかを確かめる |
| 起動してすぐ自己位置がずれる | 作り始めの印の位置と向きに置いたかを確かめる。違えば `init_x`・`init_y`・`init_yaw` か RViz の「2D Pose Estimate」で合わせる |
| 壁が二重の地図になった | ゆっくり、周回を短くして作り直す。録った rosbag から作り直せる（下） |

### 録った rosbag から地図を作り直す

機体がなくても、録った rosbag から作り直せます（`use_sim_time:=true`、ドライバと URDF の TF は rosbag のものを使う）。

```bash
ros2 launch core_launch mapping.launch.py output:=$HOME/maps/lab2.pcd use_sim_time:=true livox_driver:=false robot_state_publisher:=false
```

別のターミナルで再生します。`/tf_static` は後から繋いだノードにも届くよう、QoS を変えて再生します。

```bash
printf '/tf_static:\n  depth: 100\n  durability: transient_local\n  reliability: reliable\n  history: keep_last\n' > /tmp/tf_static_qos.yaml
```

```bash
ros2 bag play $HOME/bags/lab_<日時> --clock 100 --qos-profile-overrides-path /tmp/tf_static_qos.yaml --topics /sensing/livox/lidar /sensing/livox/imu /sensing/imu /tf_static
```

再生が終わったら、1-4 と同じように保存します。

## 実機で要確認

- Livox の IP の設定（`MID360_config.json`）と、点群（約 10Hz）・IMU（約 200Hz）が届くこと。
- URDF の Mid-360 の取り付け（`core2026.urdf.xacro` の位置と向き）が実機と合っていること。合っていないと、床の高さが
  0 からずれ、自機の点の除去もずれます。
- 研究室での自己位置の精度（止まっているときと走っているときの、地図とのずれ）。
- 手動操縦で地図を作るときに起動するものの組み合わせ（ハードウェア、車体制御、操縦アプリのブリッジ）。
- `core2026_system.xml` は車体制御（`body_controller.launch.py`）を、自分と `navigation.launch.py` の中の 2 か所で
  起動しています（以前からの構成）。`ros2 node list` で同じ名前のノードが 2 つないかを確かめてください。
