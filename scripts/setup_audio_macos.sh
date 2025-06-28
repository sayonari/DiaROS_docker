#!/bin/bash

# macOS用PulseAudio音声セットアップスクリプト
# DiaROS Docker環境でホストのマイクアクセスを有効にします

set -e

echo "🎤 macOS PulseAudioセットアップを開始します..."

# カラー出力用
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# エラーハンドリング
error_exit() {
    echo -e "${RED}❌ エラー: $1${NC}" >&2
    exit 1
}

success_msg() {
    echo -e "${GREEN}✅ $1${NC}"
}

warning_msg() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

info_msg() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

# 1. HomebrewとPulseAudioの確認
echo
info_msg "ステップ1: 依存関係の確認"

if ! command -v brew &> /dev/null; then
    error_exit "Homebrewがインストールされていません。先にHomebrewをインストールしてください。"
fi

if ! command -v pulseaudio &> /dev/null; then
    warning_msg "PulseAudioがインストールされていません。インストールを開始します..."
    brew install pulseaudio || error_exit "PulseAudioのインストールに失敗しました"
    success_msg "PulseAudioのインストールが完了しました"
else
    success_msg "PulseAudio is already installed"
fi

# 2. 既存のPulseAudioプロセスを確認・停止
echo
info_msg "ステップ2: 既存のPulseAudioプロセスの確認"

if pgrep -x "pulseaudio" > /dev/null; then
    warning_msg "既存のPulseAudioプロセスを停止しています..."
    pkill -f pulseaudio || true
    sleep 2
fi

# 3. PulseAudio設定ディレクトリの作成
echo
info_msg "ステップ3: PulseAudio設定の準備"

mkdir -p ~/.config/pulse

# 設定ファイルを作成
cat > ~/.config/pulse/default.pa << 'EOF'
#!/usr/bin/pulseaudio -nF

# Load the native protocol
load-module module-native-protocol-unix
load-module module-native-protocol-tcp auth-anonymous=1 port=4713

# Load CoreAudio driver for macOS
load-module module-coreaudio-detect

# Load other essential modules
load-module module-stream-restore
load-module module-device-restore
load-module module-default-device-restore
load-module module-rescue-streams
EOF

success_msg "PulseAudio設定ファイルを作成しました"

# 4. PulseAudioサーバーの起動
echo
info_msg "ステップ4: PulseAudioサーバーの起動"

pulseaudio --exit-idle-time=-1 --daemon -v || error_exit "PulseAudioの起動に失敗しました"

# 起動確認
sleep 3
if pulseaudio --check -v &> /dev/null; then
    success_msg "PulseAudioデーモンが正常に起動しました"
else
    error_exit "PulseAudioデーモンの起動確認に失敗しました"
fi

# 5. TCP接続の設定確認
echo
info_msg "ステップ5: TCP接続の確認"

# ポート4713の確認
if lsof -i :4713 &> /dev/null; then
    success_msg "ポート4713でPulseAudioが待機しています"
else
    warning_msg "ポート4713が開いていません。TCP接続を設定しています..."
    
    # 環境変数を設定してpactl実行
    export PULSE_SERVER=tcp:localhost:4713
    
    # 少し待ってから再試行
    sleep 2
    if lsof -i :4713 &> /dev/null; then
        success_msg "TCP接続が設定されました"
    else
        error_exit "TCP接続の設定に失敗しました"
    fi
fi

# 6. オーディオデバイスの確認
echo
info_msg "ステップ6: オーディオデバイスの確認"

# PulseAudioサーバーへの接続テスト
if pactl info &> /dev/null; then
    success_msg "PulseAudioサーバーへの接続が成功しました"
    
    # 利用可能なマイクデバイス数を取得
    source_count=$(pactl list sources short | grep -v monitor | wc -l | tr -d ' ')
    
    if [ "$source_count" -gt 0 ]; then
        success_msg "利用可能なマイクデバイス: ${source_count}個"
        echo
        info_msg "検出されたマイクデバイス:"
        pactl list sources short | grep -v monitor | while read line; do
            echo "  📱 $line"
        done
    else
        warning_msg "マイクデバイスが検出されませんでした"
    fi
else
    error_exit "PulseAudioサーバーへの接続に失敗しました"
fi

# 7. macOSマイク権限の確認メッセージ
echo
warning_msg "重要: macOSのマイク権限設定を確認してください"
echo "  1. システム設定 → プライバシーとセキュリティ → マイク"
echo "  2. 'ターミナル' にチェックを入れる"
echo "  3. 必要に応じて 'Docker Desktop' にもチェックを入れる"

# 8. 便利なaliasの提案
echo
info_msg "ステップ7: 便利なaliasの提案"

echo "以下のaliasを ~/.zshrc または ~/.bash_profile に追加することを推奨します:"
echo
echo "# DiaROS用PulseAudio管理"
echo "alias pulse-start='pulseaudio --exit-idle-time=-1 --daemon -v'"
echo "alias pulse-stop='pkill -f pulseaudio'"
echo "alias pulse-status='pulseaudio --check -v && lsof -i :4713'"
echo "alias pulse-test='pactl list sources short | grep -v monitor'"

# 9. 完了メッセージ
echo
echo "🎉 macOS PulseAudioセットアップが完了しました！"
echo
success_msg "次のステップ:"
echo "  1. macOSのマイク権限設定を確認"
echo "  2. テストスクリプトを実行: ./scripts/test_audio.sh"
echo "  3. Dockerコンテナで音声テストを実行"
echo
info_msg "Dockerコンテナ起動例:"
echo "  docker run -it --privileged -e PULSE_SERVER=tcp:host.docker.internal:4713 your_image"
echo
info_msg "トラブルシューティング:"
echo "  - PulseAudio再起動: pkill -f pulseaudio && pulseaudio --exit-idle-time=-1 --daemon -v"
echo "  - ポート確認: lsof -i :4713"
echo "  - デバイス確認: pactl list sources short"
echo
info_msg "音声接続テストを実行する場合:"
echo "  ./scripts/test_audio.sh"
echo