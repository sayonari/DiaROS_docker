#!/usr/bin/env python3
"""
M1/M2 MacのMetal Performance Shaders (MPS)サポートを確認するスクリプト
"""
import torch
import sys

def check_mps_availability():
    """MPS（Metal Performance Shaders）の利用可能性を確認"""
    print("=== PyTorch MPS (Metal) Support Check ===")
    print(f"PyTorch version: {torch.__version__}")
    print(f"Python version: {sys.version}")
    
    # MPS対応確認
    if hasattr(torch.backends, 'mps'):
        print(f"MPS backend available: {torch.backends.mps.is_available()}")
        if torch.backends.mps.is_available():
            print("✅ MPS (Metal Performance Shaders) is available!")
            print("   You can use device='mps' for M1/M2 Mac GPU acceleration")
            
            # テストテンソルを作成
            try:
                device = torch.device("mps")
                x = torch.randn(5, 3, device=device)
                print(f"✅ Successfully created tensor on MPS: {x.device}")
                
                # 簡単な演算テスト
                y = x * 2
                print("✅ MPS computation test passed")
                
            except Exception as e:
                print(f"❌ MPS test failed: {e}")
        else:
            print("❌ MPS is not available on this system")
            print("   Possible reasons:")
            print("   - Not running on Apple Silicon Mac")
            print("   - PyTorch version doesn't support MPS")
            print("   - Metal drivers not properly installed")
    else:
        print("❌ MPS backend not found in PyTorch")
        print("   Your PyTorch version may be too old (MPS requires PyTorch 1.12+)")
    
    # CUDA確認（比較のため）
    print(f"\nCUDA available: {torch.cuda.is_available()}")
    
    # 推奨デバイス
    if hasattr(torch.backends, 'mps') and torch.backends.mps.is_available():
        recommended = "mps"
    elif torch.cuda.is_available():
        recommended = "cuda"
    else:
        recommended = "cpu"
    
    print(f"\n📌 Recommended device for this system: '{recommended}'")
    
    return recommended

if __name__ == "__main__":
    check_mps_availability()