# DiaROS Docker Environment

リアルタイム音声対話システム（DiaROS）のためのDocker環境です。ROS2 Humble環境と音声処理に必要なツールを含む軽量なDockerイメージを提供します。

## 特徴

- **軽量なROS2環境**: `ros-humble-ros-base`ベースで約3-4GBのイメージサイズ
- **音声処理対応**: PyAudio、aubio、VOICEVOX Core等の音声処理ライブラリを含む
- **ローカルAI対応**: HuggingFaceの日本語モデルによる完全ローカル動作
  - 音声認識: japanese-HuBERT-base-VADLess-ASR
  - 応答生成: rinna/japanese-gpt2-small
- **デバッグツール**: rqt系ツールとrosbag2によるデータフロー監視・記録機能
- **日本語対応**: 日本語ロケール設定済み

## 必要な環境

- Docker Desktop（Docker 20.10以降）
- Docker Compose 2.0以降（Docker Desktopに含まれています）
- （macOSの場合）XQuartz（GUI表示用）
- 8GB以上のRAM推奨

**注意**: macOSやWindowsでは、Docker Desktopが起動していることを確認してください。

## セットアップ

### 1. リポジトリのクローン

```bash
git clone https://github.com/sayonari/DiaROS_docker.git
cd DiaROS_docker
```

### 2. DiaROSソースコードの配置

DiaROSのソースコードをworkspaceディレクトリにコピーまたはクローンしてください：

```bash
# 例: GitHubからクローン（方法1：workspace内の.gitkeepを削除してからクローン）
rm workspace/.gitkeep
git clone https://github.com/sayonari/DiaROS_imamoto.git workspace

# または（方法2：workspace内でクローン後、内容を移動）
cd workspace
git clone https://github.com/sayonari/DiaROS_imamoto.git temp
mv temp/* temp/.* . 2>/dev/null || true
rm -rf temp
cd ..
```

### 3. 初期セットアップ

```bash
./scripts/setup.sh
```

このスクリプトは以下を実行します：
- 環境変数ファイル（.env）の作成
- 必要なディレクトリの作成
- Dockerイメージのビルド


## 使用方法

### コンテナの起動

```bash
./scripts/run.sh
```

コンテナ内でDiaROSを起動：

```bash
# ワークスペースのビルド
cd /workspace/DiaROS_ros
colcon build --cmake-args -DCMAKE_C_FLAGS=-fPIC --packages-select interfaces
source ./install/local_setup.bash
colcon build --packages-select diaros_package
source ./install/local_setup.bash

# Pythonモジュールのインストール
cd ../DiaROS_py
python3 -m pip install . --user

# システムの起動
ros2 launch diaros_package sdsmod.launch.py
```

### モニタリングツール

別のターミナルで以下を実行：

```bash
./scripts/monitor.sh
```

利用可能なツール：
- rqt_graph: ノード間通信の可視化
- rqt_plot: データのリアルタイムプロット
- rqt_topic: トピックのモニタリング
- rqt_bag: 記録データの確認
- ros2 bag record: データの記録

### データの記録

```bash
# 全トピックを記録
ros2 bag record -a -o /recordings/session_$(date +%Y%m%d_%H%M%S)

# 特定トピックのみ記録
ros2 bag record /speech_input /recognition_result /dialog_response
```

記録されたデータは`./recordings`ディレクトリに保存されます。

## ディレクトリ構造

```
DiaROS_docker/
├── Dockerfile              # Dockerイメージ定義
├── docker-compose.yml      # Docker Compose設定
├── .env.example           # 環境変数テンプレート
├── README.md              # このファイル
├── scripts/               # ヘルパースクリプト
│   ├── setup.sh          # セットアップスクリプト
│   ├── run.sh            # 実行スクリプト
│   ├── monitor.sh        # モニタリングスクリプト
│   └── entrypoint.sh     # コンテナエントリーポイント
├── config/                # 設定ファイル（オプション）
├── workspace/             # DiaROSソースコード
└── recordings/            # bag記録ファイル
```

## トラブルシューティング

### macOSでGUIが表示されない場合

```bash
# XQuartzを起動
open -a XQuartz

# ローカルホストからの接続を許可
xhost +localhost
```

### 音声デバイスが認識されない場合

Docker設定で`privileged: true`が有効になっていることを確認してください。

### メモリ不足エラー

docker-compose.ymlのresource limitsを調整してください。

## Docker Hubでの配布

ビルド済みイメージをDocker Hubで公開する場合：

```bash
# イメージのタグ付け
docker tag diaros:slim yourusername/diaros:latest
docker tag diaros:slim yourusername/diaros:v1.0.0

# Docker Hubへプッシュ
docker login
docker push yourusername/diaros:latest
docker push yourusername/diaros:v1.0.0
```

ユーザーは以下でイメージを取得できます：

```bash
docker pull yourusername/diaros:latest
```

## ライセンス

DiaROSのライセンスに従います。詳細はDiaROSのドキュメントを参照してください。

## 貢献

イシューやプルリクエストを歓迎します。

## 参考資料

- [DiaROS公式ドキュメント](https://github.com/yourusername/DiaROS_imamoto)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Docker Documentation](https://docs.docker.com/)

## 付録：APIキーの使用（オプション）

現在のDiaROSはローカルモデルを使用するため、APIキーは不要です。ただし、以下のAPIを使用したい場合は設定できます：

### Google Cloud Speech-to-Text API（オプション）
```bash
# Google Cloud認証情報をconfigディレクトリに配置
cp /path/to/your/google_credentials.json config/
```

### A3RT Talk API（オプション）
```bash
# A3RT APIキーをconfigディレクトリに配置
cp /path/to/your/a3rt_apikey.data config/
```

これらのAPIを使用する場合は、docker-compose.ymlの環境変数セクションのコメントを解除してください。