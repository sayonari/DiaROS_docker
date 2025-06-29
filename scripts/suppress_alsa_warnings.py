#!/usr/bin/env python3
"""
ALSAの警告メッセージを抑制するヘルパースクリプト
"""
import os
import sys
import ctypes

def suppress_alsa_warnings():
    """ALSAライブラリのエラーハンドラを無効化"""
    try:
        # libasoundライブラリをロード
        asound = ctypes.cdll.LoadLibrary('libasound.so.2')
        
        # エラーハンドラを無効化
        asound.snd_lib_error_set_handler(ctypes.c_void_p(0))
        print("ALSA warnings suppressed successfully")
        return True
    except:
        print("Could not suppress ALSA warnings (libasound not found or not compatible)")
        return False

if __name__ == "__main__":
    suppress_alsa_warnings()