# core_test

共有GTestインフラパッケージです。

## テストの追加方法

1. `test/` ディレクトリに新しい `.cpp` ファイルを作成
2. `CMakeLists.txt` に登録:

```cmake
core_add_gtest(test_my_feature test/test_my_feature.cpp)
```

!!! tip
    CIの高速化のため、1テストファイル = 1ターゲットにしてください。

## テスト実行

```bash
# ローカル
colcon test --packages-select core_test
colcon test-result --verbose

# CI向け（コンソール出力あり）
colcon test --event-handlers console_direct+ --packages-select core_test
colcon test-result --verbose
```
