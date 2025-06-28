# Claude AI アシスタント設定

## 最重要事項

### 言語設定
**このリポジトリでの対話は必ず日本語で行うこと**

この設定は最優先事項であり、すべての応答、説明、コメントは日本語で記述する。

## プロジェクト固有の情報

### DiaROS Docker環境
- DiaROSは現在、ローカルモデルを使用しており、APIキーは不要
- 音声認識: HuggingFace's japanese-HuBERT-base-VADLess-ASR
- 応答生成: rinna/japanese-gpt2-small
- 音声合成: VOICEVOX

### 実行時のチェックコマンド
- リント: `npm run lint`（プロジェクトで設定されている場合）
- 型チェック: `npm run typecheck`（プロジェクトで設定されている場合）

### 既知の問題
- NumPy 2.xとaubioの互換性問題：NumPy 1.24.3に固定して使用
- macOSでのDocker音声入力：PulseAudioまたはUSB/IP経由での接続が必要