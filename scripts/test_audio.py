#!/usr/bin/env python3
"""
PulseAudio録音・再生スクリプト
dockerコンテナ内でPulseAudioを使用してマイクの録音と再生を行います
"""

import subprocess
import sys
import os
import time
import signal
import glob
from datetime import datetime
from typing import List, Dict, Optional

class PulseAudioRecorder:
    def __init__(self):
        self.current_process: Optional[subprocess.Popen] = None
        self.pulse_server = os.environ.get('PULSE_SERVER', 'tcp:host.docker.internal:4713')
        
    def get_audio_sources(self) -> List[Dict[str, str]]:
        """利用可能なオーディオソース（マイク）一覧を取得"""
        try:
            result = subprocess.run(
                ['pactl', 'list', 'sources', 'short'],
                capture_output=True,
                text=True,
                check=True,
                env={'PULSE_SERVER': self.pulse_server}
            )
            
            sources = []
            for line in result.stdout.strip().split('\n'):
                if line.strip():
                    parts = line.split('\t')
                    if len(parts) >= 4:
                        sources.append({
                            'id': parts[0],
                            'name': parts[1],
                            'driver': parts[2],
                            'format': parts[3],
                            'status': parts[4] if len(parts) > 4 else 'UNKNOWN'
                        })
            return sources
        except subprocess.CalledProcessError as e:
            print(f"Error getting audio sources: {e}")
            return []
    
    def get_audio_sinks(self) -> List[Dict[str, str]]:
        """利用可能なオーディオシンク（スピーカー）一覧を取得"""
        try:
            result = subprocess.run(
                ['pactl', 'list', 'sinks', 'short'],
                capture_output=True,
                text=True,
                check=True,
                env={'PULSE_SERVER': self.pulse_server}
            )
            
            sinks = []
            for line in result.stdout.strip().split('\n'):
                if line.strip():
                    parts = line.split('\t')
                    if len(parts) >= 4:
                        sinks.append({
                            'id': parts[0],
                            'name': parts[1],
                            'driver': parts[2],
                            'format': parts[3],
                            'status': parts[4] if len(parts) > 4 else 'UNKNOWN'
                        })
            return sinks
        except subprocess.CalledProcessError as e:
            print(f"Error getting audio sinks: {e}")
            return []
    
    def get_device_description(self, device_name: str) -> str:
        """デバイスの詳細説明を取得"""
        try:
            # デバイスの詳細情報を取得
            result = subprocess.run(
                ['pactl', 'list', 'sources'],
                capture_output=True,
                text=True,
                check=True,
                env={'PULSE_SERVER': self.pulse_server}
            )
            
            lines = result.stdout.split('\n')
            current_device = None
            device_info = {}
            
            for line in lines:
                line = line.strip()
                if line.startswith('Source #'):
                    current_device = None
                elif line.startswith('Name:'):
                    name = line.split(':', 1)[1].strip()
                    if name == device_name:
                        current_device = name
                        device_info[current_device] = {}
                elif current_device and ':' in line:
                    key, value = line.split(':', 1)
                    key = key.strip()
                    value = value.strip()
                    if key in ['Description', 'device.description', 'device.product.name']:
                        device_info[current_device][key] = value
            
            if device_name in device_info:
                desc = device_info[device_name]
                # 優先順位で説明を選択
                for key in ['Description', 'device.description', 'device.product.name']:
                    if key in desc and desc[key]:
                        return desc[key]
            
            return "不明なデバイス"
            
        except Exception:
            return "説明取得不可"
    
    def get_sink_description(self, device_name: str) -> str:
        """シンクデバイスの詳細説明を取得"""
        try:
            result = subprocess.run(
                ['pactl', 'list', 'sinks'],
                capture_output=True,
                text=True,
                check=True,
                env={'PULSE_SERVER': self.pulse_server}
            )
            
            lines = result.stdout.split('\n')
            current_device = None
            device_info = {}
            
            for line in lines:
                line = line.strip()
                if line.startswith('Sink #'):
                    current_device = None
                elif line.startswith('Name:'):
                    name = line.split(':', 1)[1].strip()
                    if name == device_name:
                        current_device = name
                        device_info[current_device] = {}
                elif current_device and ':' in line:
                    key, value = line.split(':', 1)
                    key = key.strip()
                    value = value.strip()
                    if key in ['Description', 'device.description', 'device.product.name']:
                        device_info[current_device][key] = value
            
            if device_name in device_info:
                desc = device_info[device_name]
                for key in ['Description', 'device.description', 'device.product.name']:
                    if key in desc and desc[key]:
                        return desc[key]
            
            return "不明なデバイス"
            
        except Exception:
            return "説明取得不可"
    
    def guess_device_type(self, device_name: str, description: str) -> str:
        """デバイス名と説明からデバイスタイプを推測"""
        name_lower = device_name.lower()
        desc_lower = description.lower()
        
        # 内蔵マイク/スピーカーの判定
        if any(keyword in desc_lower for keyword in ['built-in', 'internal', '内蔵', 'macbook']):
            if 'channel_1' in name_lower or '1' in device_name:
                return "内蔵マイク"
            else:
                return "内蔵スピーカー"
        
        # 外部デバイスの判定
        if any(keyword in desc_lower for keyword in ['usb', 'bluetooth', 'external', '外部']):
            return "外部デバイス"
        
        # チャンネル数による推測
        if 'channel_1__channel_2' in name_lower:
            return "ステレオデバイス"
        elif 'channel_1' in name_lower:
            return "モノラルデバイス"
        
        return "不明"
    
    def analyze_device_mapping(self) -> None:
        """ホストとコンテナのデバイスマッピングを分析して表示"""
        print("\n=== デバイスマッピング分析 ===")
        
        sources = self.get_audio_sources()
        print("\n📱 録音デバイス（マイク）:")
        print("コンテナ内でのデバイス名 → 実際のデバイス説明")
        print("-" * 80)
        
        for source in sources:
            if '.monitor' not in source['name']:
                description = self.get_device_description(source['name'])
                device_type = self.guess_device_type(source['name'], description)
                print(f"ID {source['id']}: {source['name']}")
                print(f"    → {description}")
                print(f"    → 推定タイプ: {device_type}")
                print(f"    → フォーマット: {source['format']}")
                print()
        
        sinks = self.get_audio_sinks()
        print("\n🔊 再生デバイス（スピーカー）:")
        print("コンテナ内でのデバイス名 → 実際のデバイス説明")
        print("-" * 80)
        
        for sink in sinks:
            description = self.get_sink_description(sink['name'])
            device_type = self.guess_device_type(sink['name'], description)
            print(f"ID {sink['id']}: {sink['name']}")
            print(f"    → {description}")
            print(f"    → 推定タイプ: {device_type}")
            print(f"    → フォーマット: {sink['format']}")
            print()
    
    def display_sources(self, sources: List[Dict[str, str]]) -> None:
        """オーディオソース一覧を表示（詳細説明付き）"""
        print("\n=== 利用可能なマイク（録音デバイス）===")
        print(f"{'ID':<3} {'デバイス説明':<40} {'推定タイプ':<15} {'フォーマット':<15}")
        print("-" * 80)
        
        for source in sources:
            if '.monitor' not in source['name']:
                description = self.get_device_description(source['name'])
                device_type = self.guess_device_type(source['name'], description)
                # 長い説明は切り詰める
                short_desc = description[:37] + "..." if len(description) > 40 else description
                print(f"{source['id']:<3} {short_desc:<40} {device_type:<15} {source['format']:<15}")
    
    def display_sinks(self, sinks: List[Dict[str, str]]) -> None:
        """オーディオシンク一覧を表示（詳細説明付き）"""
        print("\n=== 利用可能なスピーカー（再生デバイス）===")
        print(f"{'ID':<3} {'デバイス説明':<40} {'推定タイプ':<15} {'フォーマット':<15}")
        print("-" * 80)
        
        for sink in sinks:
            description = self.get_sink_description(sink['name'])
            device_type = self.guess_device_type(sink['name'], description)
            short_desc = description[:37] + "..." if len(description) > 40 else description
            print(f"{sink['id']:<3} {short_desc:<40} {device_type:<15} {sink['format']:<15}")
    
    def select_device(self, devices: List[Dict[str, str]], device_type: str) -> Optional[str]:
        """デバイス選択"""
        if not devices:
            print(f"利用可能な{device_type}が見つかりません。")
            return None
        
        # .monitorを除外（録音の場合）
        if device_type == "録音デバイス":
            filtered_devices = [d for d in devices if '.monitor' not in d['name']]
        else:
            filtered_devices = devices
        
        if not filtered_devices:
            print(f"利用可能な{device_type}が見つかりません。")
            return None
        
        while True:
            try:
                choice = input(f"\n{device_type}のIDを入力してください: ")
                if choice.lower() == 'q':
                    return None
                
                # IDで検索
                for device in filtered_devices:
                    if device['id'] == choice:
                        selected_device = device
                        description = ""
                        if device_type == "録音デバイス":
                            description = self.get_device_description(device['name'])
                        else:
                            description = self.get_sink_description(device['name'])
                        print(f"選択: ID {device['id']} - {description}")
                        return device['name']
                
                print(f"無効なIDです。表示されているIDを入力してください。('q'で戻る)")
            except ValueError:
                print("正しいIDを入力してください。'q'で戻る。")
            except KeyboardInterrupt:
                return None
    
    def get_audio_files(self, directory: str = ".") -> List[str]:
        """指定ディレクトリ内のオーディオファイル一覧を取得"""
        audio_extensions = ['*.wav', '*.mp3', '*.flac', '*.ogg', '*.m4a', '*.aac']
        audio_files = []
        
        for ext in audio_extensions:
            files = glob.glob(os.path.join(directory, ext))
            audio_files.extend(files)
        
        # ファイル名でソート
        audio_files.sort()
        return audio_files
    
    def select_audio_file(self, directory: str = ".") -> Optional[str]:
        """ディレクトリからオーディオファイルを選択"""
        audio_files = self.get_audio_files(directory)
        
        if not audio_files:
            print(f"ディレクトリ '{directory}' にオーディオファイルが見つかりません。")
            print("対応フォーマット: WAV, MP3, FLAC, OGG, M4A, AAC")
            return None
        
        print(f"\n=== {directory} 内のオーディオファイル ===")
        print(f"{'番号':<4} {'ファイル名':<40} {'サイズ':<10} {'更新日時':<20}")
        print("-" * 80)
        
        for i, filepath in enumerate(audio_files):
            filename = os.path.basename(filepath)
            try:
                size = os.path.getsize(filepath)
                size_str = f"{size/1024:.1f}KB" if size < 1024*1024 else f"{size/(1024*1024):.1f}MB"
                mtime = os.path.getmtime(filepath)
                mtime_str = datetime.fromtimestamp(mtime).strftime("%m/%d %H:%M:%S")
            except:
                size_str = "不明"
                mtime_str = "不明"
            
            # ファイル名が長い場合は切り詰める
            short_name = filename[:37] + "..." if len(filename) > 40 else filename
            print(f"{i:<4} {short_name:<40} {size_str:<10} {mtime_str:<20}")
        
        while True:
            try:
                choice = input(f"\nファイル番号を入力してください (0-{len(audio_files)-1}、'q'で戻る): ")
                if choice.lower() == 'q':
                    return None
                
                file_index = int(choice)
                if 0 <= file_index < len(audio_files):
                    selected_file = audio_files[file_index]
                    print(f"選択: {os.path.basename(selected_file)}")
                    return selected_file
                else:
                    print(f"無効な選択です。0-{len(audio_files)-1}の範囲で入力してください。")
            except ValueError:
                print("数値を入力してください。'q'で戻る。")
            except KeyboardInterrupt:
                return None
    
    def generate_filename(self) -> str:
        """録音日時を含むファイル名を生成"""
        now = datetime.now()
        timestamp = now.strftime("%Y%m%d_%H%M%S")
        return f"recording_{timestamp}.wav"
    
    def record_audio(self, device_name: str, duration: int = 10) -> Optional[str]:
        """指定したデバイスで録音（自動ファイル名生成）"""
        filename = self.generate_filename()
        
        print(f"\n録音開始: {filename} ({duration}秒間)")
        print(f"デバイス: {self.get_device_description(device_name)}")
        print("録音中... Ctrl+Cで停止")
        
        def signal_handler(signum, frame):
            if self.current_process:
                self.current_process.terminate()
                print("\n録音を停止しました。")
        
        signal.signal(signal.SIGINT, signal_handler)
        
        try:
            cmd = [
                'parecord',
                '--device', device_name,
                '--file-format=wav',
                '--format=s16le',
                '--rate=44100',
                '--channels=1',
                filename
            ]
            
            self.current_process = subprocess.Popen(
                cmd,
                env={'PULSE_SERVER': self.pulse_server}
            )
            
            # 指定時間待機、または手動停止まで
            try:
                self.current_process.wait(timeout=duration)
            except subprocess.TimeoutExpired:
                self.current_process.terminate()
                print(f"\n{duration}秒の録音が完了しました。")
            
            if os.path.exists(filename):
                return filename
            else:
                return None
            
        except Exception as e:
            print(f"録音エラー: {e}")
            return None
        finally:
            self.current_process = None
    
    def play_audio(self, device_name: str, filename: str) -> bool:
        """指定したデバイスで再生"""
        if not os.path.exists(filename):
            print(f"ファイルが見つかりません: {filename}")
            return False
        
        print(f"\n再生開始: {filename}")
        print("再生中... Ctrl+Cで停止")
        
        def signal_handler(signum, frame):
            if self.current_process:
                self.current_process.terminate()
                print("\n再生を停止しました。")
        
        signal.signal(signal.SIGINT, signal_handler)
        
        try:
            cmd = [
                'paplay',
                '--device', device_name,
                filename
            ]
            
            self.current_process = subprocess.Popen(
                cmd,
                env={'PULSE_SERVER': self.pulse_server}
            )
            
            self.current_process.wait()
            print("再生が完了しました。")
            return True
            
        except Exception as e:
            print(f"再生エラー: {e}")
            return False
        finally:
            self.current_process = None
    
    def show_volume_levels(self, device_name: str) -> None:
        """リアルタイムで音量レベルを表示"""
        description = self.get_device_description(device_name)
        print(f"\n音量レベル監視: {description}")
        print("マイクに向かって話してください。Ctrl+Cで停止")
        print("=" * 60)
        
        # 停止フラグ
        self.monitoring_active = True
        
        def signal_handler(signum, frame):
            print("\n\n停止中...")
            self.monitoring_active = False
            if hasattr(self, 'current_process') and self.current_process:
                try:
                    self.current_process.terminate()
                    self.current_process.wait(timeout=1)
                except:
                    try:
                        self.current_process.kill()
                    except:
                        pass
                self.current_process = None
        
        # シグナルハンドラーを設定
        original_sigint = signal.signal(signal.SIGINT, signal_handler)
        
        try:
            import struct
            
            print("音量レベル表示開始... (Ctrl+Cで停止)")
            
            while self.monitoring_active:
                try:
                    # 短時間のサンプルを取得
                    cmd = [
                        'timeout', '1',  # 1秒でタイムアウト
                        'parecord',
                        '--device', device_name,
                        '--file-format=raw',
                        '--format=s16le',
                        '--rate=44100',
                        '--channels=1'
                    ]
                    
                    self.current_process = subprocess.Popen(
                        cmd,
                        env={'PULSE_SERVER': self.pulse_server},
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE
                    )
                    
                    # 0.2秒分のデータを読み取り
                    sample_duration = 0.2
                    sample_rate = 44100
                    bytes_per_sample = 2  # 16-bit
                    bytes_to_read = int(sample_rate * sample_duration * bytes_per_sample)
                    
                    # タイムアウト付きでデータ読み取り
                    try:
                        data = self.current_process.stdout.read(bytes_to_read)
                        
                        if not self.monitoring_active:
                            break
                            
                        if len(data) >= bytes_to_read // 2:  # 最低限のデータがあれば処理
                            # バイト列を16bit整数配列に変換
                            samples = struct.unpack(f'<{len(data)//2}h', data)
                            
                            # RMS（二乗平均平方根）を計算
                            if samples:
                                rms = (sum(x*x for x in samples) / len(samples)) ** 0.5
                                # 最大値32767で正規化（16bit符号付き整数の最大値）
                                level = min(100, int((rms / 32767.0) * 100))
                                
                                # プログレスバー表示
                                bar_length = 50
                                filled_length = int(bar_length * level / 100)
                                bar = '█' * filled_length + '░' * (bar_length - filled_length)
                                
                                # 画面をクリアして同じ位置に表示
                                print(f'\r音量: [{bar}] {level:3d}%', end='', flush=True)
                            else:
                                print('\r音量: [' + '░' * 50 + ']   0%', end='', flush=True)
                        else:
                            print('\r音量: [' + '░' * 50 + ']   0% (待機中)', end='', flush=True)
                    
                    except Exception as e:
                        if self.monitoring_active:
                            print(f'\r音量監視エラー: {e}', end='', flush=True)
                    
                    # プロセス終了
                    try:
                        self.current_process.terminate()
                        self.current_process.wait(timeout=0.5)
                    except:
                        try:
                            self.current_process.kill()
                        except:
                            pass
                    finally:
                        self.current_process = None
                    
                    # 停止チェック
                    if not self.monitoring_active:
                        break
                        
                    time.sleep(0.1)  # 少し待機
                    
                except KeyboardInterrupt:
                    self.monitoring_active = False
                    break
                except Exception as e:
                    if self.monitoring_active:
                        print(f'\r監視エラー: {e}', end='', flush=True)
                        time.sleep(0.5)
                
        except KeyboardInterrupt:
            self.monitoring_active = False
        finally:
            # シグナルハンドラーを元に戻す
            signal.signal(signal.SIGINT, original_sigint)
            
            # プロセスが残っていれば終了
            if hasattr(self, 'current_process') and self.current_process:
                try:
                    self.current_process.terminate()
                    self.current_process.wait(timeout=1)
                except:
                    try:
                        self.current_process.kill()
                    except:
                        pass
                self.current_process = None
            
            print("\n\n監視を停止しました。")

def main():
    recorder = PulseAudioRecorder()
    
    print("=== PulseAudio 録音・再生ツール ===")
    print(f"PulseAudioサーバー: {recorder.pulse_server}")
    
    while True:
        print("\n=== メニュー ===")
        print("1. デバイスマッピング分析")
        print("2. 録音デバイス一覧表示")
        print("3. 再生デバイス一覧表示") 
        print("4. 録音")
        print("5. 再生")
        print("6. 音量レベル監視")
        print("7. 終了")
        
        try:
            choice = input("\n選択してください (1-7): ")
            
            if choice == '1':
                recorder.analyze_device_mapping()
                
            elif choice == '2':
                sources = recorder.get_audio_sources()
                recorder.display_sources(sources)
                
            elif choice == '3':
                sinks = recorder.get_audio_sinks()
                recorder.display_sinks(sinks)
                
            elif choice == '4':
                sources = recorder.get_audio_sources()
                recorder.display_sources(sources)
                
                device_name = recorder.select_device(sources, "録音デバイス")
                if device_name:
                    duration_str = input("録音時間（秒）を入力 (デフォルト: 10): ").strip()
                    duration = int(duration_str) if duration_str.isdigit() else 10
                    
                    result = recorder.record_audio(device_name, duration)
                    if result:
                        print(f"録音完了: {result}")
                        # ファイル情報表示
                        if os.path.exists(result):
                            size = os.path.getsize(result)
                            print(f"ファイルサイズ: {size} bytes ({size/1024:.1f} KB)")
                    else:
                        print("録音に失敗しました。")
                
            elif choice == '5':
                # ファイル選択
                selected_file = recorder.select_audio_file()
                if selected_file:
                    sinks = recorder.get_audio_sinks()
                    recorder.display_sinks(sinks)
                    
                    device_name = recorder.select_device(sinks, "再生デバイス")
                    if device_name:
                        recorder.play_audio(device_name, selected_file)
                
            elif choice == '6':
                sources = recorder.get_audio_sources()
                recorder.display_sources(sources)
                
                device_name = recorder.select_device(sources, "録音デバイス")
                if device_name:
                    recorder.show_volume_levels(device_name)
                
            elif choice == '7':
                print("終了します。")
                break
                
            else:
                print("無効な選択です。")
                
        except KeyboardInterrupt:
            print("\n\n終了します。")
            break
        except Exception as e:
            print(f"エラーが発生しました: {e}")

if __name__ == "__main__":
    main()