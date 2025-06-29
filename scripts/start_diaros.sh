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
echo -e "${GREEN}🎯 初回ビルドの確認${NC}"
echo "=================================="

# 初回ビルドが必要か確認
if [ ! -d "/workspace/DiaROS_ros/install" ]; then
    echo -e "${YELLOW}⚠️  初回ビルドが必要です。ビルドを開始します...${NC}"
    cd /workspace/DiaROS_ros
    colcon build --cmake-args -DCMAKE_C_FLAGS=-fPIC --packages-select interfaces
    source ./install/local_setup.bash
    colcon build --packages-select diaros_package
    source ./install/local_setup.bash
    echo -e "${GREEN}✅ ビルドが完了しました${NC}"
    
    # ビルド後は必ずPythonモジュールの再インストールが必要
    FORCE_REINSTALL=true
else
    echo -e "${GREEN}✅ ビルド済みです${NC}"
    FORCE_REINSTALL=false
fi

# Pythonモジュールのインストール確認
echo -e "${YELLOW}🔍 Pythonモジュールの確認...${NC}"
if [ "$FORCE_REINSTALL" = true ] || ! python3 -c "import diaros" 2>/dev/null; then
    echo -e "${YELLOW}⚠️  DiaROSモジュールのインストールが必要です...${NC}"
    cd /workspace/DiaROS_py
    python3 -m pip install . --force-reinstall
    echo -e "${GREEN}✅ モジュールのインストールが完了しました${NC}"
else
    echo -e "${GREEN}✅ DiaROSモジュールは既にインストールされています${NC}"
fi

echo ""
echo -e "${GREEN}🎯 DiaROSを起動します...${NC}"
echo "=================================="

# ROS2環境の設定
source /opt/ros/humble/setup.bash
if [ -f "/workspace/DiaROS_ros/install/local_setup.bash" ]; then
    source /workspace/DiaROS_ros/install/local_setup.bash
fi

# NumPy 1.xを強制（aubio互換性のため）
echo -e "${YELLOW}🔧 NumPy互換性の確認...${NC}"
python3 -c "import numpy; print(f'NumPy version: {numpy.__version__}')"
if python3 -c "import numpy; exit(0 if numpy.__version__.startswith('2.') else 1)" 2>/dev/null; then
    echo -e "${YELLOW}⚠️  NumPy 2.xが検出されました。aubio互換性のためNumPy 1.xにダウングレードします...${NC}"
    pip3 install --force-reinstall "numpy==1.24.3"
    echo -e "${GREEN}✅ NumPy 1.24.3にダウングレードしました${NC}"
fi

# CPU最適化の環境変数を設定（M1 Mac用）
echo -e "${YELLOW}⚙️  CPU最適化を設定中...${NC}"
# PyTorchのCPUスレッド数を最適化
export OMP_NUM_THREADS=8
export MKL_NUM_THREADS=8
export OPENBLAS_NUM_THREADS=8
# PyTorchの並列処理最適化
export TORCH_NUM_THREADS=8
# メモリアロケータの最適化
export PYTORCH_ENABLE_MPS_FALLBACK=1
echo -e "${GREEN}✅ CPU最適化設定が完了しました${NC}"

# DiaROSの起動
cd /workspace
ros2 launch diaros_package sdsmod.launch.py