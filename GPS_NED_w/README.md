# GPS_NED_w フォルダ

## 概要
OptiTrack MotiveからNatNetプロトコルでリアルタイムに剛体データを取得し、NED座標系およびGPS座標系に変換してUDP送信するPythonプログラムです。  
マルチターゲット対応で、剛体IDごとに異なるIPアドレスに送信できます。

## 主な機能

### 1. 座標変換
- **Motive座標系 → NED座標系**  
  Motiveの右手座標系(X=右, Y=上, Z=奥)をNED座標系(North, East, Down)に変換
- **NED座標系 → GPS座標系**  
  NED座標を基準GPS座標からの相対位置として緯度・経度・高度に変換

### 2. 姿勢計算
- クォータニオンからYaw角（度単位、0-360度）を計算

### 3. UDP送信
- 剛体IDごとに異なるIPアドレスに送信
- 送信先は `config.json` で設定
- 送信頻度: 50Hz（Motiveのストリーミング周波数と同期）

### 4. データ記録機能 ✨NEW
- プログラム実行中にエンターキーで記録開始/停止
- 記録内容:
  - タイムスタンプ
  - 剛体ID
  - Motiveローカル座標系の位置 (x, y, z)
  - 姿勢のクォータニオン (qx, qy, qz, qw)
- 記録停止時に自動的にCSVファイルに保存
- ファイル名: `record_YYYYMMDD_HHMMSS.csv` (記録開始時刻)
- 記録機能の有効/無効は `config.json` で設定

## ファイル構成

```
GPS_NED_w/
├── config.json              # 設定ファイル（UDP送信先、記録機能の有効/無効）
├── NatNetClient.py          # NatNetクライアント本体
├── PythonSample.py          # サンプル実行スクリプト
├── DataDescriptions.py      # データ記述クラス
├── MoCapData.py             # モーションキャプチャデータクラス
├── PythonClient.pyproj      # Visual Studioプロジェクトファイル
├── PythonClient.sln         # Visual Studioソリューションファイル
└── README.md                # このファイル
```

## 設定ファイル (config.json)

```json
{
    "udp_targets": {
        "1": "192.168.11.16",
        "2": "192.168.11.61"
    },
    "recording_enabled": true
}
```

### 設定項目

#### udp_targets
剛体IDと送信先IPアドレスのマッピング。  
送信したい剛体の数だけエントリを追加してください。

例:
- 1つだけ送信: `{"1": "192.168.11.16"}`
- 2つ送信: `{"1": "192.168.11.16", "2": "192.168.11.61"}`
- 3つ送信: `{"1": "192.168.11.16", "2": "192.168.11.61", "3": "192.168.11.70"}`

#### recording_enabled
データ記録機能の有効/無効を設定。
- `true`: 記録機能有効（エンターキーで記録開始/停止）
- `false`: 記録機能無効

## 使い方

### 1. 環境準備
```bash
pip install pyned2lla
```

### 2. 設定ファイル編集
`config.json` を編集して、送信先IPアドレスと記録機能の有効/無効を設定します。

### 3. プログラム実行
```bash
python PythonSample.py
```

### 4. データ記録（recording_enabled=trueの場合）
- プログラム起動後、エンターキーを押すと記録開始
- もう一度エンターキーを押すと記録停止してCSVファイルを保存
- CSVファイル名: `record_YYYYMMDD_HHMMSS.csv`

### 5. 停止
`Ctrl+C` で停止

## CSV出力フォーマット

| カラム名 | 説明 |
|---------|------|
| timestamp | データ受信時刻（秒） |
| rigid_body_id | 剛体ID |
| pos_x | Motiveローカル座標X |
| pos_y | Motiveローカル座標Y |
| pos_z | Motiveローカル座標Z |
| quat_x | クォータニオンX |
| quat_y | クォータニオンY |
| quat_z | クォータニオンZ |
| quat_w | クォータニオンW |

## 送信データ形式

UDP送信されるデータはPythonのpickle形式でシリアライズされた辞書です:

```python
{
    'id': 1,                           # 剛体ID
    'gps_lat': 36.0757812,             # GPS緯度（7桁精度）
    'gps_lon': 136.2132945,            # GPS経度（7桁精度）
    'gps_alt': 1.234,                  # GPS高度（3桁精度）
    'ned_position': [1.23, 4.56, -0.78],  # NED座標 [N, E, D]
    'ned_rotation': [0.0, 0.0, 0.707, 0.707],  # NEDクォータニオン [qw, qx, qy, qz]
    'yaw_deg': 123.456,                # Yaw角（度、0-360）
    'motive_position': [1.0, 2.0, 3.0],  # Motive座標
    'motive_rotation': [0.0, 0.0, 0.707, 0.707],  # Motiveクォータニオン
    'data_no': 1234,                   # データ番号
    'timestamp': 0.123,                # タイムスタンプ
    'frame_time': 12345.678            # フレーム時刻
}
```

## GPS基準座標設定

NatNetClient.py の以下の行で基準GPS座標を設定できます:

```python
self.ref_lat = 36.0757800  # 緯度（7桁精度）
self.ref_lon = 136.2132900  # 経度（7桁精度）
self.ref_alt = 0.000        # 高度（3桁精度）
```

## 注意事項

1. **ネットワーク設定**  
   Motiveの「Streaming Settings」で、このPCのIPアドレスをローカルインターフェイスとして設定してください。

2. **UDP送信先**  
   送信先のIPアドレスは、同一ネットワーク内で到達可能なアドレスを指定してください。

3. **記録データ容量**  
   50Hzで記録するため、長時間記録すると大きなファイルになります。  
   例: 1剛体で10分間記録 → 約30,000行

4. **互換性**  
   udp_targets.jsonは廃止予定です。今後はconfig.jsonをご使用ください。

## トラブルシューティング

### データが受信できない
- Motiveでストリーミングが有効になっているか確認
- ファイアウォールでポート1510, 1511が開いているか確認
- ローカルIPアドレスが正しく設定されているか確認

### UDP送信エラー
- 送信先IPアドレスが正しいか確認
- ネットワークが接続されているか確認
- 送信先ポート（デフォルト15769）が開いているか確認

### 記録が開始できない
- config.jsonで `recording_enabled: true` になっているか確認
- ターミナルでエンターキーが正しく入力できているか確認

## 開発情報

- Python 3.x
- 依存ライブラリ: pyned2lla
- サンプリング周波数: 50Hz
- UDP送信ポート: 15769
