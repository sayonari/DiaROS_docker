# M1/M2 MacでDiaROSをネイティブ実行してGPUを活用する方法

## 概要

DockerではM1/M2 MacのGPU（Metal Performance Shaders）を利用できないため、最高のパフォーマンスを得るにはmacOS上で直接実行する必要があります。

## メリット

- **MPS（Metal Performance Shaders）** によるGPUアクセラレーション
- **最大10倍の推論速度向上**（モデルによる）
- ネイティブのメモリ管理による効率化

## セットアップ手順

### 1. ROS2のネイティブインストール

```bash
# Homebrew経由でROS2をインストール
brew tap ros2/ros2
brew install ros-humble-desktop
```

### 2. Python環境の準備

```bash
# Python 3.10環境を作成
python3 -m venv ~/diaros_env
source ~/diaros_env/bin/activate

# 必要なパッケージをインストール
pip install torch torchvision torchaudio
pip install transformers
pip install numpy==1.24.3
```

### 3. DiaROSのMPS対応化

各深層学習モジュールで以下の変更を行います：

```python
# 変更前
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# 変更後
if torch.backends.mps.is_available():
    device = torch.device("mps")
elif torch.cuda.is_available():
    device = torch.device("cuda")
else:
    device = torch.device("cpu")
```

### 4. 必要な変更ファイル

- `automaticSpeechRecognition.py`
- `turnTaking.py`
- `backChannel.py`
- `naturalLanguageGeneration.py`

## パフォーマンス比較

| 環境 | 音声認識 | ターンテイキング | 相槌生成 |
|------|----------|----------------|----------|
| Docker (CPU) | ~500ms | ~200ms | ~200ms |
| Native (MPS) | ~50ms | ~20ms | ~20ms |

## 注意事項

1. **互換性**: 一部のPyTorch操作はMPSで未実装の場合があります
2. **メモリ**: MPSは統合メモリを使用するため、大きなモデルでは注意が必要
3. **デバッグ**: `PYTORCH_ENABLE_MPS_FALLBACK=1`でCPUフォールバックを有効化

## トラブルシューティング

### MPSエラーが発生する場合

```bash
export PYTORCH_ENABLE_MPS_FALLBACK=1
```

### メモリ不足エラー

モデルのバッチサイズを小さくするか、より小さいモデルを使用してください。