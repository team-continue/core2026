# 開発ワークフロー

## ブランチ規則

| 種類 | プレフィックス | 例 |
|------|-------------|-----|
| 機能追加 | `feature/` | `feature/lidar-fusion` |
| バグ修正 | `fix/` | `fix/odom-drift` |

!!! warning
    `main` ブランチへの直接コミットは禁止です。必ずPull Requestを経由してください。

## パッケージ命名規則

新しいパッケージは `core_` プレフィックスを付けてください:

```
core_<パッケージ名>
```

タブ補完で一覧表示できるため、統一されていると便利です。

## 開発フロー

### 1. ブランチ作成

```bash
git checkout main
git pull origin main
git checkout -b feature/my-feature
```

### 2. 開発・テスト

```bash
# ビルド
colcon build --symlink-install --packages-select core_my_package

# テスト
colcon test --packages-select core_my_package
colcon test-result --verbose
```

### 3. Push & PR

```bash
git push origin feature/my-feature
```

GitHub上でPull Requestを作成してください。

### 4. CI

Push時にCIが自動実行されます:

- **ビルドチェック**: 全パッケージのビルド
- **テスト実行**: 全パッケージのテスト
- **コードフォーマット**: `ament_clang_format` による自動整形（mainブランチ以外で自動コミット）

### 5. マージ

レビュー後、mainブランチにマージされます。

## CI/CD

### ビルド・テスト（ros2_ci.yml）

- **トリガー**: 全ブランチへのpush、mainへのPR、日次スケジュール
- **環境**: Ubuntu 22.04 + ROS 2 Humble
- **内容**: ビルド → テスト → コードフォーマット（自動コミット）

### ドキュメント（docs.yml）

- **トリガー**: mainブランチへのpush（docs/、mkdocs.yml変更時）
- **内容**: MkDocsビルド → GitHub Pagesデプロイ
