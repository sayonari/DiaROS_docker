#!/bin/bash
# macOS用PulseAudio起動スクリプト

echo "=== macOS PulseAudio セットアップ ==="

# PulseAudioが既に起動しているかチェック
if ps aux | grep -v grep | grep -q pulseaudio; then
    echo "PulseAudioは既に起動しています。"
else
    echo "PulseAudioを起動しています..."
    pulseaudio --load=module-native-protocol-tcp --exit-idle-time=-1 --daemon
    
    # 起動待機
    sleep 2
    
    if ps aux | grep -v grep | grep -q pulseaudio; then
        echo "✅ PulseAudioの起動に成功しました。"
    else
        echo "❌ PulseAudioの起動に失敗しました。"
        echo "以下のコマンドを手動で実行してください："
        echo "  pulseaudio --load=module-native-protocol-tcp --exit-idle-time=-1 --daemon"
        exit 1
    fi
fi

# TCP接続モジュールを追加
echo "TCP接続モジュールを設定しています..."
if pactl load-module module-native-protocol-tcp auth-anonymous=1 port=4713 2>/dev/null; then
    echo "✅ TCP接続モジュールを追加しました。"
else
    echo "ℹ️  TCP接続モジュールは既に設定されています。"
fi

# 接続確認
echo "接続状態を確認しています..."
if lsof -i :4713 > /dev/null 2>&1; then
    echo "✅ PulseAudioがポート4713で待機中です。"
    echo ""
    echo "=== セットアップ完了 ==="
    echo "DiaROSコンテナを起動できます："
    echo "  ./scripts/run.sh"
else
    echo "❌ ポート4713での接続を確認できません。"
    echo "以下を確認してください："
    echo "1. ファイアウォールがポート4713をブロックしていないか"
    echo "2. 他のプロセスがポート4713を使用していないか"
    exit 1
fi

# マイク権限の確認
echo ""
echo "📢 重要: macOSのマイク権限"
echo "初回実行時は以下の設定が必要です："
echo "  システム設定 → プライバシーとセキュリティ → マイク"
echo "  → ターミナル（またはiTerm2等）にチェック"