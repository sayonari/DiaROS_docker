# DiaROS Docker Environment

リアルタイム音声対話システム（DiaROS）のためのDocker環境です。ROS2 Humble環境と音声処理に必要なツールを含む軽量なDockerイメージを提供します。

## 目次

1. [特徴](#1-特徴)
2. [必要な環境](#2-必要な環境)
3. [セットアップ](#3-セットアップ)
4. [使用方法](#4-使用方法)
5. [ディレクトリ構造](#5-ディレクトリ構造)
6. [トラブルシューティング](#6-トラブルシューティング)
7. [Docker Hubでの配布](#7-docker-hubでの配布)
8. [ライセンス](#8-ライセンス)
9. [貢献](#9-貢献)
10. [参考資料](#10-参考資料)
11. [付録：APIキーの使用（オプション）](#11-付録apiキーの使用オプション)

## 1. 特徴

- **軽量なROS2環境**: `ros-humble-ros-base`ベースで約3-4GBのイメージサイズ
- **音声処理対応**: PyAudio、aubio、VOICEVOX Core等の音声処理ライブラリを含む
- **ローカルAI対応**: HuggingFaceの日本語モデルによる完全ローカル動作
  - 音声認識: japanese-HuBERT-base-VADLess-ASR
  - 応答生成: rinna/japanese-gpt2-small
- **デバッグツール**: rqt系ツールとrosbag2によるデータフロー監視・記録機能
- **日本語対応**: 日本語ロケール設定済み

## 2. 必要な環境

- Docker Desktop（Docker 20.10以降）
- Docker Compose 2.0以降（Docker Desktopに含まれています）
- （macOSの場合）XQuartz（GUI表示用）
- 8GB以上のRAM推奨

**注意**: macOSやWindowsでは、Docker Desktopが起動していることを確認してください。

## 3. セットアップ

### 3.1 リポジトリのクローン

```bash
git clone https://github.com/sayonari/DiaROS_docker.git
cd DiaROS_docker
```

### 3.2 macOSの事前準備（macOSユーザーのみ）

macOSでGUIアプリケーションを使用する場合、XQuartzの設定が必要です：

```bash
# XQuartzをインストール（まだの場合）
brew install --cask xquartz
# または https://www.xquartz.org/ からダウンロード

# XQuartzを起動
open -a XQuartz

# XQuartzの設定を変更
# 1. XQuartzメニュー → 環境設定 → セキュリティタブ
# 2. 「ネットワーク・クライアントからの接続を許可」にチェック
# 3. XQuartzを再起動（必要な場合）

# ローカルホストからの接続を許可
# まずDISPLAY環境変数を設定
export DISPLAY=:0
xhost +localhost

# もし「xhost: command not found」エラーが出る場合は：
/opt/X11/bin/xhost +localhost

# もし「unable to open display」エラーが出る場合は：
# 1. XQuartzが起動していることを確認
# 2. 新しいターミナルを開いて再試行
# 3. または以下を実行：
export DISPLAY=:0
export PATH="/opt/X11/bin:$PATH"
xhost +localhost
```

### 3.3 DiaROSソースコードの配置

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

### 3.4 初期セットアップ

```bash
./scripts/setup.sh
```

このスクリプトは以下を実行します：
- 環境変数ファイル（.env）の作成
- 必要なディレクトリの作成
- Dockerイメージのビルド

**注意**: macOSの場合、setup.shが`.env`ファイルのDISPLAY変数を自動的に`host.docker.internal:0`に設定します。


## 4. 使用方法

### 4.1 コンテナの起動

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
python3 -m pip install .

# システムの起動
ros2 launch diaros_package sdsmod.launch.py
```

**注意**: 初回起動時には、HuggingFaceへのログインが必要な場合があります（[トラブルシューティング](#62-huggingfaceモデルのアクセスエラー)参照）。

### 4.2 モニタリングツール

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

### 4.3 データの記録

```bash
# 全トピックを記録
ros2 bag record -a -o /recordings/session_$(date +%Y%m%d_%H%M%S)

# 特定トピックのみ記録
ros2 bag record /speech_input /recognition_result /dialog_response
```

記録されたデータは`./recordings`ディレクトリに保存されます。

## 5. ディレクトリ構造

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

## 6. トラブルシューティング

### 6.1 Pythonパッケージの不足エラー

`ModuleNotFoundError: No module named 'librosa'`などのエラーが出る場合：

```bash
# コンテナ内で以下を実行
pip3 install librosa soundfile pydub playsound webrtcvad aubio huggingface-hub

# NumPyの互換性問題がある場合
pip3 install --force-reinstall "numpy<2"

# GStreamerエラーの場合
apt-get update && apt-get install -y \
    gstreamer1.0-python3-plugin-loader \
    python3-gst-1.0 \
    ffmpeg
```

または、Dockerイメージを再ビルド：
```bash
docker-compose build --no-cache
```

### 6.2 HuggingFaceモデルのアクセスエラー

DiaROSの起動時に以下のようなエラーが出る場合：
```
OSError: You are trying to access a gated repo.
huggingface_hub.errors.LocalTokenNotFoundError: Token is required
```

これはモデルがアクセス制限されているためです。以下の方法で解決できます：

**方法1: HuggingFaceでモデルの利用規約に同意する（推奨）**

1. [HuggingFace](https://huggingface.co/)でアカウントを作成
2. 以下のモデルページにアクセスして利用規約に同意：
   - [音声認識モデル](https://huggingface.co/SiRoZaRuPa/japanese-HuBERT-base-VADLess-ASR-RSm)
   - [ターンテイキングモデル](https://huggingface.co/Yosato/Wav2Vec2-deberta-keep-continue)（必要な場合）
3. [アクセストークン](https://huggingface.co/settings/tokens)を作成
4. コンテナ内でHuggingFaceにログイン：
   ```bash
   pip install huggingface-hub
   huggingface-cli login
   # トークンを入力
   ```

**方法2: モデルを変更する（代替案）**

`workspace/DiaROS_py/diaros/automaticSpeechRecognition.py`の177行目を編集：

```python
# 変更前
MODEL_ID = 'SiRoZaRuPa/japanese-HuBERT-base-VADLess-ASR-RSm'

# 変更後（rinnaの公開モデルを使用）
MODEL_ID = 'rinna/japanese-hubert-base'
```

変更後、Pythonモジュールを再インストール：
```bash
cd /workspace/DiaROS_py
python3 -m pip install .
```

### 6.3 macOSでGUIが表示されない場合

セットアップ手順の「2. macOSの事前準備」を実行していない場合は、以下を確認してください：

1. **XQuartzが起動しているか確認**
   ```bash
   pgrep -x XQuartz || open -a XQuartz
   ```

2. **xhostの設定を確認**
   ```bash
   # XQuartzのパスが通っていない場合は：
   /opt/X11/bin/xhost +localhost
   # または
   export PATH="/opt/X11/bin:$PATH" && xhost +localhost
   ```

3. **環境変数の確認**
   ```bash
   cat .env | grep DISPLAY
   # DISPLAY=host.docker.internal:0 になっていることを確認
   ```

4. **それでも表示されない場合はコンテナを再起動**
   ```bash
   docker-compose down
   docker-compose up -d
   ```

### 6.4 音声デバイスが認識されない場合

`OSError: No Default Input Device Available`エラーが出る場合：

**macOSの場合:**
1. macOSのシステム環境設定 → セキュリティとプライバシー → プライバシー → マイク
2. Docker Desktopがマイクへのアクセスを許可されているか確認
3. 許可されていない場合は、チェックボックスをオンにする
4. Docker Desktopを再起動

**Linuxの場合:**
```bash
# ホストでオーディオグループに追加
sudo usermod -a -G audio $USER
# 再ログインまたは再起動
```

Docker設定で`privileged: true`が有効になっていることも確認してください。

### 6.5 メモリ不足エラー

docker-compose.ymlのresource limitsを調整してください。

## 7. Docker Hubでの配布

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

## 8. ライセンス

DiaROSのライセンスに従います。詳細はDiaROSのドキュメントを参照してください。

## 9. 貢献

イシューやプルリクエストを歓迎します。

## 10. 参考資料

- [DiaROS公式ドキュメント](https://github.com/yourusername/DiaROS_imamoto)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Docker Documentation](https://docs.docker.com/)

## 11. 付録：APIキーの使用（オプション）

現在のDiaROSはローカルモデルを使用するため、APIキーは不要です。ただし、以下のAPIを使用したい場合は設定できます：

### 11.1 Google Cloud Speech-to-Text API（オプション）
```bash
# Google Cloud認証情報をconfigディレクトリに配置
cp /path/to/your/google_credentials.json config/
```

### 11.2 A3RT Talk API（オプション）
```bash
# A3RT APIキーをconfigディレクトリに配置
cp /path/to/your/a3rt_apikey.data config/
```

これらのAPIを使用する場合は、docker-compose.ymlの環境変数セクションのコメントを解除してください。