#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Motive専用 Pixhawk パラメータ設定スクリプト
- 内蔵・外付けコンパスの使用をすべて無効化 (COMPASS_USE=0, COMPASS_USE2=0, COMPASS_USE3=0)
- EKF3のヨーソースを GPS (Motiveのヨー角) に設定 (EK3_SRC1_YAW=2)
"""

from pymavlink import mavutil
import time
import sys

def set_parameter(master, param_name, param_value, param_type):
    """MAVLink経由でパラメータを設定するヘルパー関数"""
    print(f"パラメータ送信中: {param_name} -> {param_value}")
    
    # 送信バッファのクリアを期待して少し待つ
    time.sleep(0.1)
    
    # パラメータ設定メッセージを送信
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_name.encode('utf-8'),
        param_value,
        param_type
    )
    
    # 設定反映を確認するために待機して読み出しを試みる
    # (実機環境に合わせてパラメータ要求メッセージを送信)
    time.sleep(0.2)
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        param_name.encode('utf-8'),
        -1
    )
    
    # 返答を待機するループ
    start_time = time.time()
    while time.time() - start_time < 2.0:
        msg = master.recv_match(type='PARAM_VALUE', blocking=False)
        if msg:
            ret_name = msg.param_id
            if isinstance(ret_name, bytes):
                ret_name = ret_name.decode('utf-8', errors='ignore')
            ret_name = ret_name.strip('\x00')
            
            if ret_name == param_name:
                print(f"✓ 設定完了確認: {param_name} = {msg.param_value}")
                return True
        time.sleep(0.05)
        
    print(f"⚠ 反映確認タイムアウト: {param_name} (値が書き換わっているかGCS等でご確認ください)")
    return False

def main():
    print("=" * 60)
    print(" Motive専用 Pixhawk パラメータ設定")
    print(" （コンパス無効化 & GPSヨー設定）")
    print("=" * 60)
    
    # 接続設定の選択
    print("接続先を選択してください:")
    print(" 1: Serial 接続 (/dev/ttyAMA0, 1000000 baud, RTS/CTS有効) ※デフォルト")
    print(" 2: USB 接続 (/dev/ttyACM0, 115200 baud)")
    print(" 3: Windows Serial 接続 (例: COM3, 115200 baud)")
    
    choice = input("選択 (1-3, デフォルト 1): ").strip()
    
    device = '/dev/ttyAMA0'
    baud = 1000000
    rtscts = True
    
    if choice == '2':
        device = '/dev/ttyACM0'
        baud = 115200
        rtscts = False
    elif choice == '3':
        port = input("COMポート名を入力してください (例: COM3): ").strip()
        device = port if port else 'COM3'
        baud = 115200
        rtscts = False

    print(f"\n✓ MAVLink接続を開始します: {device} (BaudRate: {baud})")
    try:
        master = mavutil.mavlink_connection(device, baud=baud, rtscts=rtscts)
    except Exception as e:
        print(f"❌ 接続エラー: {e}")
        sys.exit(1)
        
    print("ハートビート信号を待機中...")
    master.wait_heartbeat()
    print(f"✓ 接続完了 (System ID: {master.target_system}, Component ID: {master.target_component})")
    
    # 1. コンパスの無効化 (COMPASS_USE = 0)
    set_parameter(master, "COMPASS_USE", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
    set_parameter(master, "COMPASS_USE2", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
    set_parameter(master, "COMPASS_USE3", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
    
    # 2. EKF3 ヨーソースを GPS (Motiveのヨー角) に設定 (EK3_SRC1_YAW = 2)
    set_parameter(master, "EK3_SRC1_YAW", 2, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
    
    print("\n" + "=" * 60)
    print("設定が完了しました。")
    print("※ パラメータの変更を完全に反映させるため、Pixhawk を再起動してください。")
    print("=" * 60)

if __name__ == "__main__":
    main()
