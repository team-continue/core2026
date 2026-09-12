# DepthAI SDK

このROSパッケージをビルドする前に、固定バージョンのDepthAI 3.9.0 SDKをインストールしてください。

bash scripts/depthAI_install.sh

このスクリプトは、自身の配置場所を基準にパスを解決するため、どのディレクトリからでも実行できます。

DepthAIのソースコードは
third_party/depthai-core

ビルド用ファイルは
third_party/depthai-build

SDKのインストール先は
third_party/depthai_install

に配置されます。

これらの生成ファイルはGitにはコミットしません。
COLCON_IGNORE によって、colconがthird_party配下のパッケージを探索しないようにしています。

インストーラスクリプトでは apt-get を使用します。
一般ユーザーで実行する場合は sudo を使用します。

システム依存パッケージがすでにインストール済みの場合は、

DEPTHAI_SKIP_APT=1

を設定することで、aptによるインストール処理をスキップできます。

メモリの少ないマシンでは、

DEPTHAI_BUILD_JOBS=1

を設定してください。
デフォルトの並列ビルド数は2です。

このスクリプトでは、USBアクセス権限やudevルールの変更は行いません。
既存のOAK-D用ハードウェア設定はそのまま使用されます。

CI環境では、rosdepの実行後、colcon buildの前にこのインストーラを実行する必要があります。

ローカル環境のパスだけを変更しても、GitHub Actionsのrunner上にはDepthAI SDKはインストールされません。

使用するDepthAI SDKのrevisionはスクリプト内で固定されています。
また、提案しているCIキャッシュキーには、このインストーラスクリプトのハッシュ値を含めます。

******************************************************************

Install the pinned DepthAI 3.9.0 SDK before building this ROS package:

```bash
bash scripts/depthAI_install.sh
```

The script resolves paths relative to itself, so it can be called from any directory.
Sources go in `third_party/depthai-core`, build files in `third_party/depthai-build`,
and the SDK in `third_party/depthai_install`. Generated files are not committed.
`COLCON_IGNORE` prevents colcon from discovering third-party packages.

The installer uses apt-get (sudo on a non-root machine). Set
`DEPTHAI_SKIP_APT=1` if system dependencies are already installed and
`DEPTHAI_BUILD_JOBS=1` on machines with limited memory (default: 2).
It does not change USB permissions or udev rules. Existing hardware setup remains valid.

CI must run the installer after rosdep and before colcon build. Changing a local
path alone does not install the SDK on GitHub runners. The SDK revision is pinned
in the script; the proposed CI cache key includes the installer hash.
