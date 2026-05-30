# GPS_NED_w_timestamp 設計資料

## 1. 概要

Motive（モーションキャプチャシステム）から受信した剛体の位置・姿勢データを、UDP経由で各ドローンのRaspiに送信するSDK。

Raspi側では受信したデータをMAVLink（`GPS_INPUT` / `SYSTEM_TIME`）に変換し、ArduPilotに渡すことでドローンのローカルポジショニングを実現する。

---

## 2. システム構成

```
┌─────────────────────────────────────────────────┐
│  Motive PC (Windows)                            │
│                                                 │
│  Motive ──(NatNet)──▶ PythonSample.py          │
│                         │                       │
│                         │ NatNetClient          │
│                         │ (50Hz: GPS_INPUT用)   │
│                         │ (struct埋め込み)      │
│                         │                       │
│            ┌────────────┼────────────┐          │
│            ▼            ▼            ▼          │
│       UDP socket   UDP socket   UDP socket      │
│       (永続)       (永続)       (永続)          │
│            │            │            │          │
└────────────┼────────────┼────────────┼──────────┘
             │            │            │
       Ethernet/WiFi  (異なるIPアドレス)
             │            │            │
             ▼            ▼            ▼
┌────────────┴────────────┴────────────┴──────────┐
│  Drone Network                                   │
│                                                 │
│  ┌──────────┐   ┌──────────┐   ┌──────────┐    │
│  │ Raspi A  │   │ Raspi B  │   │ Raspi C  │    │
│  │(192.168. │   │(192.168. │   │(192.168. │    │
│  │  x.x.10) │   │  x.x.20) │   │  x.x.30) │    │
│  │ 剛体 ID=1│   │ 剛体 ID=2│   │ 剛体 ID=3│    │
│  │          │   │          │   │          │    │
│  │UDP受信   │   │UDP受信   │   │UDP受信   │    │
│  │  ↓       │   │  ↓       │   │  ↓       │    │
│  │MAVLink→  │   │MAVLink→  │   │MAVLink→  │    │
│  │FlightCtrl│   │FlightCtrl│   │FlightCtrl│    │
│  └──────────┘   └──────────┘   └──────────┘    │
│       │              │              │            │
│       ▼              ▼              ▼            │
│  ┌──────────┐   ┌──────────┐   ┌──────────┐    │
│  │Drone A   │   │Drone B   │   │Drone C   │    │
│  │(gps_id=1)│   │(gps_id=2)│   │(gps_id=3)│    │
│  └──────────┘   └──────────┘   └──────────┘    │
│                                                 │
└─────────────────────────────────────────────────┘
```

### 設計上のポイント

- **各ドローンには1台のRaspiが搭載されている**
- **各Raspiには自機の剛体データのみが届く**（SDK側でIPルーティング）
- 不要なデータをドローンに送らないことで、ネットワーク帯域とRaspiのCPU負荷を最小化

---

## 3. データフロー

### 3.1 全体の流れ

```
Motive (50Hz) → NatNetClient → 剛体IDでルーティング → UDP → Raspi → MAVLink → ArduPilot
```

### 3.2 送信データの種類と周期

| データ種別 | MAVLinkメッセージ | SDK→Raspi送信周期 | Raspi→FC送信周期 | 内容 |
|-----------|-------------------|-------------------|--------------------------|------|
| GPS位置情報 | `GPS_INPUT` | **50Hz** | **15Hz**（最新値定期送信） | 緯度経度高度(GPS変換済み) Yaw角 |
| システム時刻 | `SYSTEM_TIME` | **50Hz**（GPSデータに埋め込み） | **15Hz**（毎回間引きなし） | `time.time_ns()`を秒単位に変換したUnix時刻 |

> **補足**: `SYSTEM_TIME`は独立UDPパケットではなくstructの`unix_time_sec`として埋め込み。

### 3.3 タイムスタンプの考え方

- SDK側ではstructバイナリ(§4.3参照)に`unix_time_sec`を埋め込み50HzでRaspiに送信
- Raspi側では受信した`unix_time_sec`をそのままMAVLink `SYSTEM_TIME` として毎フレーム(15Hz) ArduPilotに転送（間引きなし）
- `SYSTEM_TIME` は15Hzの毎回送信で十分な精度

> **参考**: ArduPilot の `AP_GPS_MAV` は `GPS_INPUT.time_usec` を参照せず、受信時刻を自前で記録する。
> `SYSTEM_TIME` は `system_time_unix_usec` フィールドを持ち、ArduPilot 全体の時刻基準として使用される。

---

## 4. UDP通信仕様

### 4.1 ルーティング方式

**SDK側ルーティング（現行方式）**

`config.json` の `udp_targets` で剛体IDと送信先IPアドレスをマッピングする。

```json
{
  "udp_targets": {
    "1": "192.168.1.10",
    "2": "192.168.1.20",
    "3": "192.168.1.30"
  },
  "udp_port": 15769
}
```

- 剛体 ID=1 のデータ → `192.168.1.10:15769` に送信
- 剛体 ID=2 のデータ → `192.168.1.20:15769` に送信
- マッピングにない剛体IDのデータは送信しない

### 4.2 ソケット管理

`__init__`で宛先IPごとに永続UDPソケットを1つだけ作成し全フレームで使い回す。

```python
# __init__ で作成（1回のみ）
self.udp_sockets = {}  # {target_ip: socket}
for rigid_id, target_ip in self.udp_targets.items():
    if target_ip not in self.udp_sockets:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sockets[target_ip] = sock

# send_udp_data では sendto のみ
def send_udp_data(self, data, target_ip, rigid_body_id):
    try:
        sock = self.udp_sockets[target_ip]
        sock.sendto(data, (target_ip, self.udp_port))
    except Exception as e:
        print(f"UDP send error: {e}")
```

- UDP はコネクションレスなので、1つのソケットで任意の宛先に `sendto()` 可能
- ただし、宛先IPごとにソケットを分けることで、送信先の識別とエラーハンドリングが容易になる
- 破棄は `shutdown()` 時に一括で行う

### 4.3 データフォーマット（struct バイナリ）

UDP通信には`struct`による23バイト固定長バイナリを使用。

| offset | サイズ | 型 | フィールド名 | 内容 |
|--------|--------|-----|-------------|------|
| 0 | 1byte | `uint8` | `rigid_body_id` | 剛体ID |
| 1 | 4byte | `int32` | `lat_e7` | 緯度×1e7 |
| 5 | 4byte | `int32` | `lon_e7` | 経度×1e7 |
| 9 | 4byte | `int32` | `alt_mm` | 高度 [mm] |
| 13 | 2byte | `uint16` | `yaw_cdeg` | Yaw角×100 [cdeg] |
| 15 | 8byte | `float64` | `unix_time_sec` | SDK生成Unix時刻 [秒] |

> **注意**: 位置高度はSDK側で`ned_to_gps()`によりGPS座標に変換済み。Raspi側では変換不要。

**パック/アンパック例**

```python
# SDK側（パック）
import struct
packed = struct.pack('<BiiiHd', rigid_body_id, lat_e7, lon_e7, alt_mm, yaw_cdeg, unix_time_sec)
# 23バイト固定長

# Raspi側（アンパック）
rigid_body_id, lat_e7, lon_e7, alt_mm, yaw_cdeg, unix_time_sec = struct.unpack('<BiiiHd', packed)
```

---

## 5. 送信周期の制御

### 5.1 SDK側（Motive PC）

50Hzでフレーム受信のたびに`motive_to_ned()` → `ned_to_gps()` でGPS変換 → `struct.pack` でバイナリ化 → `send_udp_data()` で送信する。`SYSTEM_TIME`は独立パケットではなく`unix_time_sec`として`struct`に埋め込む。`send_udp_data()`は永続ソケット(§4.2参照)を使用する。

```python
def __unpack_rigid_body(self, ...):
    import struct, time
    
    # Motive座標 → NED変換
    ned_x, ned_y, ned_z = self.motive_to_ned(pos_x, pos_y, pos_z)
    # NED → GPS変換
    lat_e7, lon_e7, alt_mm = self.ned_to_gps(ned_x, ned_y, ned_z)
    # クォータニオン → Yaw角 [deg]
    yaw_deg = self.quaternion_to_yaw_degrees(qx, qy, qz, qw)
    # Yaw角 [cdeg]
    yaw_cdeg = int(yaw_deg * 100)
    # Unix時刻 [秒]
    unix_time_sec = time.time_ns() / 1e9
    
    # struct パック (23バイト固定長)
    packed = struct.pack('<BiiiHd',
        new_id, lat_e7, lon_e7, alt_mm, yaw_cdeg, unix_time_sec)
    
    self.send_udp_data(packed, target_ip, new_id)
```

### 5.2 Raspi側

Raspi側ではUDPでstructバイナリを受信し、最新データを15Hz周期でArduPilotにMAVLink送信する。SYSTEM_TIMEは間引かず毎回送信。

| データ | SDK→Raspi周期 | Raspi→ArduPilot周期 | 実装方針 |
|--------|--------------|---------------------|---------|
| `GPS_INPUT` | 50Hz(UDP受信) | **15Hz** | structアンパック後そのままMAVLink送信 |
| `SYSTEM_TIME` | 50Hz(struct埋め込み) | **15Hz** | unix_time_secを抽出して毎回MAVLink SYSTEM_TIME送信（間引きなし） |

```python
# Raspi側 15Hz定期送信
import struct, time
from pymavlink import mavutil

# MAVLink接続のセットアップ
mav = mavutil.mavlink_connection('/dev/serial0', baud=921600)

class PeriodicSender:
    """15Hz (66.67ms間隔) の定期送信ループ"""
    def __init__(self):
        self.interval = 1.0 / 15.0  # 66.67ms
        self.latest_data = None
        self._running = True

    def update_data(self, lat_e7, lon_e7, alt_mm, yaw_cdeg, unix_time_sec):
        """UDP受信時に最新データを更新（alt_mm は mm 単位）"""
        self.latest_data = (lat_e7, lon_e7, alt_mm, yaw_cdeg, unix_time_sec)

    def run(self):
        """15Hz定期送信ループ"""
        next_time = time.perf_counter()
        while self._running:
            if self.latest_data is not None:
                lat_e7, lon_e7, alt_mm, yaw_cdeg, unix_time_sec = self.latest_data
                # 毎回 SYSTEM_TIME 送信（間引きなし）
                send_system_time(mav, unix_time_sec)
                # 毎回 GPS_INPUT 送信
                send_gps_input(mav, lat_e7, lon_e7, alt_mm, yaw_cdeg, unix_time_sec)
            
            next_time += self.interval
            sleep_time = next_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)

# UDP受信ループ（structバイナリ受信 → 最新データ更新のみ）
sender = PeriodicSender()
# sender.run() は別スレッドで起動

while True:
    data, addr = sock.recvfrom(4096)
    # struct.unpack: <B=uint8 rigid_body_id, i=int32 lat_e7, i=lon_e7, i=alt_mm,
    #                H=uint16 yaw_cdeg, d=float64 unix_time_sec
    rigid_body_id, lat_e7, lon_e7, alt_mm, yaw_cdeg, unix_time_sec = \
        struct.unpack('<BiiiHd', data[:23])
    sender.update_data(lat_e7, lon_e7, alt_mm, yaw_cdeg, unix_time_sec)
```

> 間引きカウンタ（`gps_frame_count % 5`）は廃止。SYSTEM_TIMEは毎回送信される。

---

## 6. Raspi側の処理（参考）

Raspi側の受信スクリプトは以下の流れで動作する：

1. UDP ソケットで `struct` バイナリデータ（23バイト固定長）を受信
2. `struct.unpack('<BiiiHd', data[:23])` で全フィールドを一度にアンパック（structは単一フォーマット）
3. 受信後すぐに `GPS_INPUT` 送信 + `SYSTEM_TIME` 送信（間引きなし）
4. MAVLinkメッセージを Flight Controller にシリアル経由で送信

```python
# Raspi側 擬似コード
import socket, struct

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 15769))

while True:
    data, addr = sock.recvfrom(4096)
    rigid_body_id, lat_e7, lon_e7, alt_mm, yaw_cdeg, unix_time_sec = \
        struct.unpack('<BiiiHd', data[:23])
    
    # GPS_INPUT 送信（間引きなし）
    send_mavlink_gps_input(lat_e7, lon_e7, alt_mm, yaw_cdeg, unix_time_sec)
    # SYSTEM_TIME 送信（間引きなし）
    send_mavlink_system_time(unix_time_sec)
```

---

## 7. 設定ファイル (config.json)

```json
{
  "udp_targets": {
    "1": "192.168.1.10",
    "2": "192.168.1.20",
    "3": "192.168.1.30"
  },
  "udp_port": 15769,
  "system_time_divider": 5,
  "recording_enabled": false
}
```

| 項目 | 説明 |
|------|------|
| `udp_targets` | 剛体ID → 送信先IPアドレスのマッピング |
| `udp_port` | UDP送信先ポート番号 |
| `system_time_divider` | （予備。現在は未使用） |
| `recording_enabled` | `true` でCSV記録機能が有効化（`NatNetClient.__init__` で読み込み）。デフォルト: `false` |

---

## 8. ファイル構成

```
GPS_NED_w_timestamp/
├── DESIGN.md          ← 本資料
├── README.md          ← 使用方法
├── config.json        ← 設定ファイル
├── PythonSample.py    ← エントリポイント（キーボード監視・記録制御）
├── NatNetClient.py    ← NatNet通信 + UDP送信（ソケット永続化）
├── MoCapData.py       ← MoCapデータパース
└── DataDescriptions.py ← データ記述子
```

---

## 9. CSV記録機能

### 9.1 概要

SDKは Motive から受信した剛体の生データ（位置・姿勢）をCSVファイルに記録する機能を備えている。記録機能の有効/無効は `config.json` の `recording_enabled` で制御し、有効時は Enter キーのトグル操作で記録の開始/停止を行う。

### 9.2 設定 (config.json)

設定項目の詳細は [§7 設定ファイル](#7-設定ファイル-configjson) を参照。`recording_enabled` は §7 の config.json に統合されている。

### 9.3 ファイル形式

#### 保存先

```
~/Downloads/record_YYYYMMDD_HHMMSS.csv
```

記録開始時刻をファイル名に使用する（例: `record_20260531_143025.csv`）。

#### CSVカラム構成

| カラム | 型 | 説明 | 記録元 |
|--------|-----|------|--------|
| `timestamp` | int | SDK生成タイムスタンプ（`time.time_ns()`、ナノ秒単位のUnix時刻） | `time.time_ns()` |
| `rigid_body_id` | int | 剛体ID | `new_id` |
| `pos_x` | float | Motive座標系 X（北） | `pos[0]` |
| `pos_y` | float | Motive座標系 Y（上） | `pos[1]` |
| `pos_z` | float | Motive座標系 Z（東） | `pos[2]` |
| `quat_x` | float | クォータニオン X成分 | `rot[0]` |
| `quat_y` | float | クォータニオン Y成分 | `rot[1]` |
| `quat_z` | float | クォータニオン Z成分 | `rot[2]` |
| `quat_w` | float | クォータニオン W成分 | `rot[3]` |

> **注意**: CSVには Motive から受信した**生データ**（変換前）が記録される。座標変換（`motive_to_ned()` + `ned_to_gps()`）は UDP 送信用（`GPS_INPUT`）であり、CSV 記録には適用されない。
> **注意**: `timestamp` 列は **SDK側で `time.time_ns()` により生成** される。Motive からはフレームのタイムスタンプが送られてこないため（§3.3参照）、システムクロックに基づくナノ秒精度の時刻を記録する。

#### サンプルデータ

```csv
timestamp,rigid_body_id,pos_x,pos_y,pos_z,quat_x,quat_y,quat_z,quat_w
1717084800000000000,1,0.123,1.234,2.345,0.001,0.002,0.003,0.999
1717084800020000000,1,0.125,1.236,2.347,0.001,0.002,0.003,0.999
1717084800040000000,1,0.127,1.238,2.349,0.001,0.002,0.003,0.999
```

各CSVファイルには連続したフレームデータのみが記録される。フライト区切りは別ファイルで管理する。

### 9.4 記録操作（Enterキートグル）

`PythonSample.py` の `keyboard_monitor()`（`msvcrt` による非ブロッキングキー入力監視スレッド）で、Enter キー押下による **2ステートトグル** を実装する。

```
         Enter（開始）              Enter（停止＆保存）
  待機 ──────────────────▶ 記録中 ──────────────────▶ 待機
 (idle)                    (recording)                 (idle)
                              │
                              └── 次のEnterで新規CSVとして再度記録開始
```

| 遷移 | アクション |
|------|-----------|
| 待機 → 記録中 | `NatNetClient.start_recording()` を呼び出し、記録バッファを初期化 |
| 記録中 → 待機 | `NatNetClient.stop_recording()` を呼び出し、CSVファイルを保存 |

- Enter を押すたびに新規CSVファイルが生成される（記録開始時刻がファイル名になる）
- 記録中にもう一度 Enter を押すと即座にCSV保存され、次の Enter で新しい記録が始まる
- 全0行の区切りマーカーは挿入しない（ファイル分割で代用）

### 9.5 実装詳細

#### 記録バッファ

`NatNetClient.recording_data`（`list`）に1行ずつデータを蓄積する。各要素は `[timestamp, rigid_body_id, pos_x, pos_y, pos_z, quat_x, quat_y, quat_z, quat_w]` のリスト。

#### データ追加

`NatNetClient.__unpack_rigid_body()` 内（L561-569）で、`is_recording` が `True` の場合に毎フレーム（50Hz）データを追加する:
- タイムスタンプには `time.time_ns()` でSDK側のシステム時刻をナノ秒単位で記録する（Motiveのタイムスタンプは使用しない）

```python
if self.is_recording:
    self.recording_data.append([
        time.time_ns(),
        new_id,
        pos[0], pos[1], pos[2],
        rot[0], rot[1], rot[2], rot[3]
    ])
```

#### 記録開始 (`start_recording`, L266-281)

- `recording_enabled` が `False` の場合はエラーメッセージを表示して終了
- `is_recording = True`、`recording_data = []` に初期化、開始時刻を記録

#### 記録停止とCSV保存 (`stop_recording`, L283-331)

- `csv.writer` を使用して `~/Downloads/` にCSVファイルを出力
- ファイル名は `record_YYYYMMDD_HHMMSS.csv`（開始時刻から生成）
- 先頭行にヘッダーを書き込み、続けて全データ行を出力

### 9.6 関連機能との関係

| 機能 | データ内容 | 座標系 | 用途 |
|------|-----------|--------|------|
| **CSV記録**（本セクション） | Motive生データ（位置・姿勢） | Motive座標系（X=北, Y=上, Z=東） | デバッグ・解析 |
| **UDP送信**（§3, §4） | struct 23byteバイナリ（`rigid_body_id`, `lat_e7`, `lon_e7`, `alt_mm`, `yaw_cdeg`, `unix_time_sec`） | GPS座標（`ned_to_gps()` 変換済み） | ドローン制御 |

CSV 記録は UDP 送信とは独立して動作し、互いに影響しない。

---

## 10. 変更履歴

| 日付 | 変更内容 |
|------|---------|
| 2026-05-31 | UDP通信をpickleからstruct 23byte固定長バイナリに変更。GPS座標変換(SDK側で`ned_to_gps()` → `lat_e7/lon_e7/alt_mm`)、`unix_time_sec`をstructに埋め込み。Raspi→FC周期をGPS_INPUT 15Hz/SYSTEM_TIME 15Hz(間引きなし)に統一。SYSTEM_TIMEは独立パケット廃止しstruct埋め込み。§4.2のタイトルと説明を実装仕様に変更。config.jsonから`system_time_divider`を削除。 |
| 2026-05-31 | Enterキー操作を3ステートトグルから2ステートトグルに簡略化（全0行マーカー廃止、Enterごとに新規CSV生成）。CSVサンプルのtimestampを time.time_ns() 整数形式に修正 |
| 2026-05-31 | CSV記録の timestamp を Motive公式タイムスタンプから SDK生成(`time.time_ns()`) に修正（Motiveがタイムスタンプを送信しない実装に合わせた設計変更） |
| 2026-05-31 | 初版作成。Motiveからのタイムスタンプ未送信に対応。`time.time_ns()` による `SYSTEM_TIME` 送信方式に変更。SDK側はGPS/SYSTEM_TIMEともに50Hzで間引きなし送信。Raspi側でSYSTEM_TIMEをGPS 5回に1回（3Hz）に間引いてArduPilot転送。`GPS_INPUT` から時刻フィールドを削除。ソケット永続化の設計を追加。SDK側ルーティング方式を正式採用。CSV記録機能セクションを追加。 |
