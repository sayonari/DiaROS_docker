#!/bin/bash
# PyAudioをPulseAudioサポート付きでインストールするスクリプト

echo "🎤 PyAudioのPulseAudioサポートインストール"
echo "=========================================="

# 必要なパッケージのインストール
echo "1. 依存パッケージのインストール..."
apt-get update
apt-get install -y \
    build-essential \
    portaudio19-dev \
    libportaudio2 \
    libportaudiocpp0 \
    libasound2-dev \
    libasound2-plugins \
    libpulse-dev \
    pulseaudio \
    pulseaudio-utils

# ALSAプラグインの確認
echo ""
echo "2. ALSAプラグインの確認..."
if find /usr/lib -name "libasound_module_pcm_pulse.so" 2>/dev/null | grep -q pulse; then
    echo "✅ ALSA PulseAudioプラグインが見つかりました"
    find /usr/lib -name "libasound_module_pcm_pulse.so" 2>/dev/null
else
    echo "❌ ALSA PulseAudioプラグインが見つかりません"
fi

# PyAudioの再インストール
echo ""
echo "3. PyAudioの再インストール..."
pip3 uninstall -y pyaudio
pip3 install --no-cache-dir pyaudio

# 設定ファイルの作成
echo ""
echo "4. ALSA設定ファイルの作成..."
cat > /etc/asound.conf << 'EOF'
pcm.!default {
    type pulse
    hint {
        show on
        description "Default ALSA Output (PulseAudio)"
    }
}
ctl.!default {
    type pulse
}
EOF

# ユーザー設定も作成
cat > ~/.asoundrc << 'EOF'
pcm.!default {
    type pulse
}
ctl.!default {
    type pulse
}
EOF

echo "✅ ALSA設定ファイルを作成しました"

# 環境変数の設定
echo ""
echo "5. 環境変数の設定..."
export PULSE_LATENCY_MSEC=30
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/alsa-lib:$LD_LIBRARY_PATH

# テスト
echo ""
echo "6. インストールの確認..."
python3 -c "
import pyaudio
p = pyaudio.PyAudio()
print(f'PyAudio version: {pyaudio.__version__}')
print(f'PortAudio version: {pyaudio.pa.__version__}')
print(f'Device count: {p.get_device_count()}')
p.terminate()
" || echo "❌ PyAudioのテストに失敗しました"

echo ""
echo "✨ インストール完了"
echo ""
echo "次のステップ:"
echo "1. コンテナを再起動して設定を反映"
echo "   docker-compose restart"
echo "2. 再度コンテナに入る"
echo "   docker exec -it diaros_container bash"
echo "3. テストスクリプトを実行"
echo "   python3 /workspace/scripts/test_pyaudio_pulse.py"