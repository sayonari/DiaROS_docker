#!/bin/bash
# DiaROS起動スクリプト

# 色付きの出力
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}🚀 DiaROS起動準備${NC}"
echo "=================================="

# power_calibration.wavファイルの存在確認
if [ ! -f "/workspace/power_calibration.wav" ]; then
    echo -e "${YELLOW}⚠️  power_calibration.wavファイルが見つかりません。コピーします...${NC}"
    cp /workspace/DiaROS_py/power_calibration.wav /workspace/
    echo -e "${GREEN}✅ ファイルをコピーしました${NC}"
fi

# HuggingFaceトークンの確認
if [ -z "$HF_TOKEN" ]; then
    echo -e "${YELLOW}⚠️  HuggingFaceトークンが設定されていません${NC}"
    echo "ターンテイキング機能を使用する場合は、以下のいずれかの方法でトークンを設定してください："
    echo "1. export HF_TOKEN=your_token"
    echo "2. huggingface-cli login"
    echo ""
    read -p "今すぐHuggingFace CLIでログインしますか？ (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        huggingface-cli login
    else
        echo -e "${YELLOW}⚠️  ターンテイキング機能は使用できません${NC}"
    fi
else
    echo -e "${GREEN}✅ HuggingFaceトークンが設定されています${NC}"
fi

# 音声デバイスの設定確認
if [ -f "/workspace/config/audio_device.conf" ]; then
    source /workspace/config/audio_device.conf
    echo -e "${GREEN}✅ 音声デバイス設定を読み込みました: AUDIO_DEVICE_INDEX=$AUDIO_DEVICE_INDEX${NC}"
fi

echo ""
echo -e "${GREEN}🎯 DiaROSを起動します...${NC}"
echo "=================================="

# ROS2環境の設定
source /opt/ros/humble/setup.bash
source /workspace/DiaROS_ros/install/local_setup.bash

# DiaROSの起動
cd /workspace
ros2 launch diaros_package sdsmod.launch.py