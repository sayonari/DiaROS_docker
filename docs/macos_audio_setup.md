# MacOSでDockerコンテナからマイクアクセスする手順

## 前提条件
- macOS（M1/M2/Intel対応）
- Docker Desktop
- Homebrew

## 手順1: ホスト側でPulseAudioをセットアップ

### 1.1 PulseAudioのインストール
```bash
# HomebrewでPulseAudioをインストール
brew install pulseaudio
```

### 1.2 macOSのマイク権限設定
1. **システム設定** → **プライバシーとセキュリティ** → **マイク**
2. **ターミナル**（またはPulseAudioを実行するアプリ）にチェックを入れる
3. 必要に応じて**Docker Desktop**にもマイク権限を付与

### 1.3 PulseAudioサーバーの起動
```bash
# TCP接続を許可してPulseAudioデーモンを起動
pulseaudio --load=module-native-protocol-tcp --exit-idle-time=-1 --daemon --verbose

# 起動確認
pulseaudio --check -v
```

### 1.4 TCP接続モジュールの追加
```bash
# 認証なしTCP接続を許可（ポート4713）
pactl load-module module-native-protocol-tcp auth-anonymous=1 port=4713

# 設定確認
lsof -i :4713
```

## 手順2: Dockerコンテナの起動

### 2.1 コンテナ起動コマンド
```bash
docker run -it \
  --privileged \
  -e PULSE_SERVER=tcp:host.docker.internal:4713 \
  your_container_image
```

### 2.2 Docker Composeを使用する場合
```yaml
version: '3.8'
services:
  your_service:
    image: your_container_image
    stdin_open: true
    tty: true
    privileged: true
    environment:
      - PULSE_SERVER=tcp:host.docker.internal:4713
```

## 手順3: コンテナ内での確認・テスト

### 3.1 必要パッケージのインストール
```bash
# コンテナ内で実行
apt update
apt install -y alsa-utils pulseaudio-utils
```

### 3.2 接続確認
```bash
# PulseAudioサーバーへの接続テスト
pactl info

# 利用可能なマイクデバイス一覧
pactl list sources short
```

### 3.3 録音テスト
```bash
# 10秒間の録音テスト（デバイスIDは実際の値に置き換え）
parecord --device=1 --file-format=wav test.wav &
sleep 10
kill %1

# ファイル確認
ls -la test.wav
```

## 手順4: 便利なPython録音スクリプト（オプション）

以下のPythonスクリプトでデバイス選択と録音が簡単にできます：

```python
#!/usr/bin/env python3
import subprocess
import os
from datetime import datetime

def get_audio_sources():
    result = subprocess.run(['pactl', 'list', 'sources', 'short'], 
                          capture_output=True, text=True)
    sources = []
    for line in result.stdout.strip().split('\n'):
        if line.strip() and '.monitor' not in line:
            parts = line.split('\t')
            sources.append({'id': parts[0], 'name': parts[1]})
    return sources

def record_audio(device_id, duration=10):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"recording_{timestamp}.wav"
    
    cmd = ['parecord', '--device', device_id, '--file-format=wav', filename]
    process = subprocess.Popen(cmd)
    process.wait()
    
    return filename

# 使用例
sources = get_audio_sources()
for source in sources:
    print(f"ID: {source['id']}, Name: {source['name']}")

# ID指定で録音
filename = record_audio('1', 10)  # IDは実際の値に置き換え
print(f"録音完了: {filename}")
```

## トラブルシューティング

### 問題1: `pactl`で "Connection refused" エラー
```bash
# PulseAudioプロセス確認
ps aux | grep pulseaudio

# 必要に応じて再起動
pkill -f pulseaudio
pulseaudio --load=module-native-protocol-tcp --exit-idle-time=-1 --daemon -v
```

### 問題2: デバイスが認識されない
```bash
# ポート4713が開いているか確認
lsof -i :4713

# macOSのマイク権限を再確認
# システム設定 → プライバシーとセキュリティ → マイク
```

### 問題3: 録音ファイルが空
```bash
# 実際のマイクデバイスを確認（.monitorではないもの）
pactl list sources short | grep -v monitor

# マイクの音量レベル確認
pactl list sources | grep -A 10 "Name: 1"
```

## 自動化スクリプト例

### ホスト側セットアップスクリプト（setup_audio.sh）
```bash
#!/bin/bash
echo "PulseAudioセットアップ開始..."

# PulseAudioが実行中かチェック
if ! pgrep -x "pulseaudio" > /dev/null; then
    echo "PulseAudioを起動しています..."
    pulseaudio --load=module-native-protocol-tcp --exit-idle-time=-1 --daemon -v
    sleep 2
fi

# TCP接続モジュール追加
echo "TCP接続を設定しています..."
pactl load-module module-native-protocol-tcp auth-anonymous=1 port=4713 2>/dev/null || true

# 確認
if lsof -i :4713 > /dev/null; then
    echo "✅ PulseAudioがポート4713で待機中"
    echo "✅ セットアップ完了"
else
    echo "❌ セットアップに失敗しました"
    exit 1
fi
```

### Docker実行スクリプト例（run_with_audio.sh）
```bash
#!/bin/bash
echo "音声対応Dockerコンテナを起動しています..."

docker run -it \
  --privileged \
  -e PULSE_SERVER=tcp:host.docker.internal:4713 \
  --name audio_container \
  your_image_name \
  bash
```

## 重要なポイント

1. **セキュリティ**: `--privileged`フラグは必要最小限の使用に留める
2. **ポート**: 4713番ポートがファイアウォールで blocked されていないことを確認
3. **権限**: macOSのマイク権限設定は必須
4. **デバイス選択**: `.monitor`デバイスは出力のモニタリング用なので録音には使用しない

この方法により、MacOSのDocker Desktopからでもホストのマイクデバイスにアクセスできます。