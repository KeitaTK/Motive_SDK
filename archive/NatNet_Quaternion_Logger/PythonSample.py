import sys
import time
import csv
import os
from datetime import datetime
from NatNetClient import NatNetClient
import threading

# ダウンロードフォルダのパスを取得
downloads_folder = os.path.join(os.path.expanduser('~'), 'Downloads')

def receive_rigid_body_frame(new_id, position, rotation):
    """Rigid bodyデータを受信したときのコールバック"""
    pass

def main():
    # NatNetクライアントの作成
    streaming_client = NatNetClient()
    streaming_client.set_client_address("127.0.0.1")
    streaming_client.set_server_address("127.0.0.1")
    streaming_client.set_use_multicast(True)
    
    # Rigid bodyのリスナーを設定
    streaming_client.rigid_body_listener = receive_rigid_body_frame
    
    # クライアントを起動
    is_running = streaming_client.run()
    if not is_running:
        print("ERROR: Could not start streaming client.")
        try:
            sys.exit(1)
        except SystemExit:
            print("...")
        finally:
            print("exiting")
    
    # 接続確認
    time.sleep(1)
    if streaming_client.connected():
        print("\nSuccessfully connected to Motive!")
        print("Press ENTER to start recording...")
        print("Press ENTER again to stop recording.")
        print("Press Ctrl+C to exit the program.\n")
    else:
        print("ERROR: Could not connect to Motive.")
        streaming_client.shutdown()
        return
    
    recording_count = 0
    
    try:
        while True:
            # エンター入力を待つ（記録開始）
            input()
            
            # 記録開始
            recording_count += 1
            streaming_client.start_recording()
            print(f"\n=== Recording #{recording_count} started ===")
            print("Press ENTER to stop recording...")
            
            # エンター入力を待つ（記録停止）
            input()
            
            # 記録停止
            streaming_client.stop_recording()
            
            # データを取得
            recorded_data = streaming_client.get_recorded_data()
            
            if len(recorded_data) > 0:
                # CSVファイル名を生成（Quaternion_日時.csv）
                timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
                csv_filename = f"Quaternion_{timestamp_str}.csv"
                csv_path = os.path.join(downloads_folder, csv_filename)
                
                # CSVファイルに書き込み
                with open(csv_path, 'w', newline='', encoding='utf-8') as csvfile:
                    fieldnames = ['timestamp', 'id', 'local_x', 'local_y', 'local_z', 
                                  'quat_x', 'quat_y', 'quat_z', 'quat_w', 'data_no', 'data_time']
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    
                    writer.writeheader()
                    for data in recorded_data:
                        writer.writerow(data)
                
                print(f"\n=== Recording #{recording_count} stopped ===")
                print(f"Saved {len(recorded_data)} frames to: {csv_path}")
            else:
                print(f"\n=== Recording #{recording_count} stopped ===")
                print("No data was recorded.")
            
            print("\nPress ENTER to start next recording...")
            print("Press Ctrl+C to exit the program.\n")
    
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        streaming_client.shutdown()
        print("Program terminated.")

if __name__ == "__main__":
    main()
