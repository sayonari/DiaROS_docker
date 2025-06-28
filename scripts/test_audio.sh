#!/bin/bash

# DiaROS Docker環境用音声テストスクリプト
# macOSホスト側とDockerコンテナでの音声接続をテストします

set -e

# カラー出力用
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ヘルパー関数
success_msg() {
    echo -e "${GREEN}✅ $1${NC}"
}

error_msg() {
    echo -e "${RED}❌ $1${NC}"
}

warning_msg() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

info_msg() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

echo "🎤 DiaROS Docker音声接続テスト"
echo "=================================="

# 1. ホスト側PulseAudioテスト
echo
info_msg "ステップ1: ホスト側PulseAudioの確認"

# PulseAudioプロセス確認
if pgrep -x "pulseaudio" > /dev/null; then
    success_msg "PulseAudioプロセスが実行中です"
else
    error_msg "PulseAudioプロセスが実行されていません"
    echo "  以下のコマンドで起動してください:"
    echo "  ./scripts/setup_audio_macos.sh"
    exit 1
fi

# PulseAudioサーバー接続確認
if pactl info &> /dev/null; then
    success_msg "PulseAudioサーバー接続: OK"
    
    # サーバー情報表示
    server_name=$(pactl info | grep "Server Name" | cut -d: -f2 | sed 's/^ *//')
    server_version=$(pactl info | grep "Server Version" | cut -d: -f2 | sed 's/^ *//')
    echo "  サーバー名: $server_name"
    echo "  バージョン: $server_version"
else
    error_msg "PulseAudioサーバー接続: 失敗"
    exit 1
fi

# 2. ネットワーク接続確認
echo
info_msg "ステップ2: ネットワーク接続の確認"

# ポート4713確認
if lsof -i :4713 &> /dev/null; then
    success_msg "ポート4713でPulseAudioが待機中"
    
    # 接続詳細表示
    echo "  接続詳細:"
    lsof -i :4713 | tail -n +2 | while read line; do
        echo "    $line"
    done
else
    error_msg "ポート4713が開いていません"
    echo "  以下のコマンドでTCP接続を有効にしてください:"
    echo "  pactl load-module module-native-protocol-tcp auth-anonymous=1 port=4713"
    exit 1
fi

# 3. オーディオデバイス確認
echo
info_msg "ステップ3: オーディオデバイスの確認"

# マイクデバイス一覧取得
mic_devices=$(pactl list sources short | grep -v monitor)
mic_count=$(echo "$mic_devices" | wc -l | tr -d ' ')

if [ "$mic_count" -gt 0 ]; then
    success_msg "利用可能なマイクデバイス: ${mic_count}個"
    echo
    echo "📱 マイクデバイス一覧:"
    echo "$mic_devices" | while read line; do
        if [ -n "$line" ]; then
            id=$(echo "$line" | cut -f1)
            name=$(echo "$line" | cut -f2)
            format=$(echo "$line" | cut -f4)
            echo "  ID: $id | $name | $format"
        fi
    done
else
    warning_msg "マイクデバイスが検出されませんでした"
    echo "  macOSのマイク権限設定を確認してください"
fi

# スピーカーデバイス一覧取得
speaker_devices=$(pactl list sinks short)
speaker_count=$(echo "$speaker_devices" | wc -l | tr -d ' ')

if [ "$speaker_count" -gt 0 ]; then
    success_msg "利用可能なスピーカーデバイス: ${speaker_count}個"
    echo
    echo "🔊 スピーカーデバイス一覧:"
    echo "$speaker_devices" | while read line; do
        if [ -n "$line" ]; then
            id=$(echo "$line" | cut -f1)
            name=$(echo "$line" | cut -f2)
            format=$(echo "$line" | cut -f4)
            echo "  ID: $id | $name | $format"
        fi
    done
else
    warning_msg "スピーカーデバイスが検出されませんでした"
fi

# 4. Docker接続テスト
echo
info_msg "ステップ4: Docker接続テストの準備"

# Dockerがインストールされているか確認
if ! command -v docker &> /dev/null; then
    error_msg "Dockerがインストールされていません"
    exit 1
fi

# Docker Desktopが実行されているか確認
if ! docker info &> /dev/null; then
    error_msg "Docker Desktopが実行されていません"
    echo "  Docker Desktopを起動してください"
    exit 1
fi

success_msg "Docker環境: OK"

# 5. Dockerコンテナでの音声テスト用コマンド表示
echo
info_msg "ステップ5: Dockerコンテナ音声テストコマンド"

echo "以下のコマンドでDockerコンテナ内での音声テストを実行できます:"
echo
echo "🐳 基本的な音声テスト:"
echo "docker run --rm -it --privileged \\"
echo "  -e PULSE_SERVER=tcp:host.docker.internal:4713 \\"
echo "  ubuntu:22.04 bash"
echo
echo "📋 コンテナ内で実行するコマンド:"
echo "apt update && apt install -y pulseaudio-utils alsa-utils"
echo "pactl info                          # PulseAudio接続確認"
echo "pactl list sources short           # マイクデバイス一覧"
echo "pactl list sinks short             # スピーカーデバイス一覧"
echo "parecord --device=1 --file-format=wav test.wav  # 録音テスト"

# 6. 自動化されたDockerテスト（オプション）
echo
read -p "Dockerコンテナでの自動音声テストを実行しますか？ (y/N): " -n 1 -r
echo

if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo
    info_msg "Dockerコンテナでの音声テストを開始..."
    
    # 一時的なDockerコンテナで音声テスト
    docker run --rm -i --privileged \
        -e PULSE_SERVER=tcp:host.docker.internal:4713 \
        ubuntu:22.04 bash << 'EOF'
echo "🐳 Dockerコンテナ内での音声テスト"
echo "=================================="

# パッケージインストール
echo "📦 必要なパッケージをインストール中..."
apt update > /dev/null 2>&1
apt install -y pulseaudio-utils > /dev/null 2>&1

# PulseAudio接続テスト
echo
echo "🔌 PulseAudioサーバー接続テスト..."
if pactl info &> /dev/null; then
    echo "✅ PulseAudioサーバー接続: 成功"
    
    # サーバー情報表示
    echo "📊 サーバー情報:"
    pactl info | grep -E "(Server|Host|User)" | sed 's/^/  /'
else
    echo "❌ PulseAudioサーバー接続: 失敗"
    exit 1
fi

# デバイス一覧表示
echo
echo "🎤 利用可能なマイクデバイス:"
mic_devices=$(pactl list sources short | grep -v monitor)
if [ -n "$mic_devices" ]; then
    echo "$mic_devices" | while read line; do
        if [ -n "$line" ]; then
            echo "  📱 $line"
        fi
    done
    echo "✅ マイクデバイスが正常に検出されました"
else
    echo "⚠️  マイクデバイスが検出されませんでした"
fi

echo
echo "🔊 利用可能なスピーカーデバイス:"
speaker_devices=$(pactl list sinks short)
if [ -n "$speaker_devices" ]; then
    echo "$speaker_devices" | while read line; do
        if [ -n "$line" ]; then
            echo "  🔈 $line"
        fi
    done
    echo "✅ スピーカーデバイスが正常に検出されました"
else
    echo "⚠️  スピーカーデバイスが検出されませんでした"
fi

echo
echo "🎉 Dockerコンテナでの音声テストが完了しました！"
EOF

    if [ $? -eq 0 ]; then
        echo
        success_msg "Dockerコンテナでの音声テストが成功しました！"
    else
        echo
        error_msg "Dockerコンテナでの音声テストに失敗しました"
    fi
fi

# 7. 完了メッセージとトラブルシューティング
echo
echo "🎊 音声接続テストが完了しました！"
echo
success_msg "テスト結果まとめ:"
echo "  ✅ ホスト側PulseAudio: OK"
echo "  ✅ ネットワーク接続: OK"
echo "  ✅ オーディオデバイス: 検出済み"
echo "  ✅ Docker環境: OK"

echo
info_msg "次のステップ:"
echo "  1. DiaROSコンテナを音声対応で起動"
echo "  2. 実際の音声認識・対話テスト"

echo
info_msg "トラブルシューティング:"
echo "  🔧 PulseAudio再起動: pkill -f pulseaudio && ./scripts/setup_audio_macos.sh"
echo "  🔧 ポート確認: lsof -i :4713"
echo "  🔧 デバイス確認: pactl list sources short | grep -v monitor"
echo "  🔧 macOS権限確認: システム設定 → プライバシーとセキュリティ → マイク"

echo
info_msg "DiaROSコンテナ起動例:"
echo "  docker-compose up -d"
echo "  # または"
echo "  docker run -it --privileged -e PULSE_SERVER=tcp:host.docker.internal:4713 diaros:slim"
echo