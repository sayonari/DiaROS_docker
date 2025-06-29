#!/bin/bash
# ALSA設定ファイルのバックアップと不要なPCMカード設定のコメントアウト

echo "ALSAエラーメッセージを削減する設定を適用します..."

# バックアップディレクトリを作成
mkdir -p /etc/alsa/backup

# alsa.confのバックアップ（まだ存在しない場合）
if [ ! -f /etc/alsa/backup/alsa.conf.original ]; then
    cp /usr/share/alsa/alsa.conf /etc/alsa/backup/alsa.conf.original
fi

# カスタムALSA設定を作成
cat > /etc/asound.conf << 'EOF'
# Docker環境用のシンプルなALSA設定
# 不要なPCMデバイスを無効化してエラーメッセージを削減

# デフォルトデバイスをPulseAudioに設定
pcm.!default {
    type pulse
    fallback "null"
}

ctl.!default {
    type pulse
    fallback "null"
}

# 不要なPCMデバイスを無効化
pcm.null {
    type null
}

ctl.null {
    type null
}
EOF

echo "ALSA設定が完了しました。"