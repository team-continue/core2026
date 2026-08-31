# core_test

## Purpose

各パッケージが個別にテスト環境を構築すると、CMakeの記述が重複しCIの設定も分散します。GTestのセットアップを1か所に集約し、`core_add_gtest()` の1行でテストを追加できるようにするための共有テストインフラパッケージです。ノードは含みません。

## Inner-workings

`cmake/` に定義された `core_add_gtest()` マクロが、GTestのリンク・ament登録・テスト実行ターゲットの生成をまとめて行います。各パッケージはこのマクロを呼ぶだけでテストを追加できます。

## テストの追加方法

1. `test/` ディレクトリに新しい `.cpp` ファイルを作成
2. `CMakeLists.txt` に登録:

```cmake
core_add_gtest(test_my_feature test/test_my_feature.cpp)
```

!!! tip
    CIの高速化のため、1テストファイル = 1ターゲットにしてください。ターゲットが分かれていると並列実行でき、失敗箇所の特定も容易になります。

## テスト実行

```bash
# ローカル
colcon test --packages-select core_test
colcon test-result --verbose

# CI向け（コンソール出力あり）
colcon test --event-handlers console_direct+ --packages-select core_test
colcon test-result --verbose
```

## Assumptions / Known limits

- GTest（C++）のみを対象とします。Pythonパッケージのテストは各パッケージ側で `pytest` を使って個別に設定してください。
- ハードウェアやEtherCAT接続を必要とするテストは対象外です。実機依存の確認は、[core_hardware](../core_hardware/index.md)の構成に合わせて個別に行ってください。
