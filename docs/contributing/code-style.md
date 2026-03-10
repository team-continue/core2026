# コードスタイル

## C/C++

### ament_clang_format

C/C++ ファイルは `ament_clang_format` で自動整形されます。

CIでのpush時にフォーマッタが自動実行され、変更があればブランチに自動コミットされます（mainブランチを除く）。

### ローカルでの実行

```bash
# フォーマットチェック
source /opt/ros/humble/setup.bash
ament_clang_format --check src/my_file.cpp

# 自動修正
ament_clang_format --reformat src/my_file.cpp
```

### 対象ファイル

拡張子: `.c`, `.cc`, `.cpp`, `.h`, `.hpp`

## Python

ROS2の標準的なPythonスタイル（PEP 8）に従ってください。

## パッケージ構成

新しいパッケージを追加する場合の標準構成:

```
core_my_package/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── core_my_package/
│       └── my_node.hpp
├── src/
│   ├── my_node.cpp
│   └── node.cpp          # main()
├── launch/
│   └── my_package.launch.py
├── param/
│   └── default_params.yaml
├── test/
│   └── test_my_feature.cpp
└── README.md
```
