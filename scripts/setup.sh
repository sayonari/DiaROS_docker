#!/bin/bash

# DiaROS Docker Setup Script

set -e

echo "==================================="
echo "DiaROS Docker Setup"
echo "==================================="

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed. Please install Docker first."
    exit 1
fi

# Check if Docker daemon is running
if ! docker info &> /dev/null; then
    echo "Error: Docker daemon is not running."
    echo "Please start Docker Desktop and try again."
    exit 1
fi

# Check if docker-compose is installed
if ! command -v docker-compose &> /dev/null && ! docker compose version &> /dev/null; then
    echo "Error: docker-compose is not installed. Please install docker-compose first."
    exit 1
fi

# Create .env file from template if it doesn't exist
if [ ! -f .env ]; then
    echo "Creating .env file from template..."
    cp .env.example .env
    
    # Detect OS and set appropriate DISPLAY variable
    if [[ "$OSTYPE" == "darwin"* ]]; then
        echo "Detected macOS. Setting DISPLAY for XQuartz..."
        sed -i '' 's/DISPLAY=:0/DISPLAY=host.docker.internal:0/' .env
        echo "Please ensure XQuartz is installed and running."
        echo "Run: xhost +localhost"
    fi
else
    echo ".env file already exists. Skipping..."
fi

# Create config directory structure
echo "Creating configuration directories..."
mkdir -p config
mkdir -p recordings
mkdir -p workspace

# Note about API credentials (optional)
echo ""
echo "Note: DiaROS now uses local models by default"
echo "API credentials are optional for backward compatibility"

# macOS音声セットアップ関数
setup_macos_audio() {
    echo "🎤 macOS音声環境をセットアップしています..."
    
    # PulseAudioインストール確認
    if ! command -v pulseaudio &> /dev/null; then
        echo "⚠️  PulseAudioがインストールされていません。"
        echo "以下のコマンドでインストールしてください:"
        echo "brew install pulseaudio"
        echo ""
        echo "インストール後、以下のコマンドで音声セットアップを実行してください:"
        echo "./scripts/setup_audio_macos.sh"
        return 1
    fi
    
    echo "✅ PulseAudioが検出されました"
    echo ""
    echo "macOS音声セットアップオプション:"
    echo "  1. 自動セットアップを実行"
    echo "  2. 手動でセットアップ"
    echo "  3. スキップ"
    echo ""
    
    read -p "選択してください (1-3): " -n 1 -r
    echo ""
    
    case $REPLY in
        1)
            echo "🚀 自動セットアップを開始します..."
            if [ -f "scripts/setup_audio_macos.sh" ]; then
                chmod +x scripts/setup_audio_macos.sh
                ./scripts/setup_audio_macos.sh
            else
                echo "❌ setup_audio_macos.sh が見つかりません"
                echo "リポジトリの構成を確認してください"
                return 1
            fi
            ;;
        2)
            echo "📖 手動セットアップの手順:"
            echo "  1. ./scripts/setup_audio_macos.sh を実行"
            echo "  2. ./scripts/test_audio.sh でテスト"
            echo "  3. docker-compose up -d でコンテナ起動"
            ;;
        3)
            echo "⏭️  音声セットアップをスキップしました"
            echo "後で以下のコマンドで実行できます:"
            echo "  ./scripts/setup_audio_macos.sh"
            ;;
        *)
            echo "❌ 無効な選択です。音声セットアップをスキップします"
            ;;
    esac
    
    echo ""
    echo "📚 音声関連のスクリプト:"
    echo "  🔧 セットアップ: ./scripts/setup_audio_macos.sh"
    echo "  🧪 テスト: ./scripts/test_audio.sh"
    echo "  📖 詳細: README.md の「6.4 音声デバイスが認識されない場合」"
}

# macOS特有のセットアップ
if [[ "$OSTYPE" == "darwin"* ]]; then
    echo ""
    echo "🍎 macOSが検出されました。"
    echo ""
    setup_macos_audio
    
    echo ""
    echo "🎯 macOS特有の注意事項:"
    echo "  📱 システム設定 → プライバシーとセキュリティ → マイク で権限設定"
    echo "  🐳 Docker Desktopが起動していることを確認"
    echo "  🔊 音声テスト: ./scripts/test_audio.sh"
fi

# Build Docker image
echo ""
echo "Building Docker image..."
echo "This may take several minutes on first run..."
docker-compose build

echo ""
echo "==================================="
echo "Setup completed!"
echo "==================================="
echo ""
echo "Next steps:"
echo "1. Copy your DiaROS source code to ./workspace/"
if [[ "$OSTYPE" == "darwin"* ]]; then
    echo "2. (macOS) 音声セットアップ: ./scripts/setup_audio_macos.sh"
    echo "3. (macOS) 音声テスト: ./scripts/test_audio.sh"
    echo "4. Run: ./scripts/run.sh"
else
    echo "2. Run: ./scripts/run.sh"
fi
echo ""
echo "Available commands:"
echo "  ./scripts/run.sh     - Start the container"
echo "  ./scripts/monitor.sh - Monitor the system"
if [[ "$OSTYPE" == "darwin"* ]]; then
    echo "  ./scripts/setup_audio_macos.sh - Setup audio (macOS)"
    echo "  ./scripts/test_audio.sh        - Test audio connection"
fi
echo ""