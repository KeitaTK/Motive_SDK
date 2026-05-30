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
│                         │ (50Hz: SYSTEM_TIME用) │
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

| データ種別 | MAVLinkメッセージ | SDK→Raspi送信周期 | Raspi→ArduPilot送信周期 | 内容 |
|-----------|-------------------|-------------------|--------------------------|------|
| GPS位置情報 | `GPS_INPUT` | **50Hz** | **50Hz**（そのまま転送） | 位置(NED)、速度、姿勢（不使用） |
| システム時刻 | `SYSTEM_TIME` | **50Hz** | **3Hz**（5回に1回に間引き） | `time.time_ns()` で取得した現在時刻 |

### 3.3 タイムスタンプの考え方

- **Motiveからは時刻データが送られてこない**
- そのため、SDK側で `time.time_ns()` を使用してシステム時刻を取得し、`SYSTEM_TIME` に格納して送信する
- `GPS_INPUT` の `time_usec` フィールドは **ArduPilot側で一切使用されない**ため、デフォルト値（0）のままでよい
- SDK側では GPS / SYSTEM_TIME ともに **50Hz で送信**（間引きなし）。Raspi側で SYSTEM_TIME のみ **5回に1回（3Hz）** に間引いて ArduPilot に転送する
- SYSTEM_TIME は ArduPilot の時刻同期用であり、高頻度である必要はない。3Hz で十分

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

### 4.2 ソケット管理（改善設計）

**現在の問題点**: 毎フレーム `socket()` → `sendto()` → `close()` を実行している。
50Hz × 剛体数（例: 3機）= **秒間150回のソケット生成/破棄**が発生し、非効率。

**改善案**: `__init__` で宛先IPごとに **永続UDPソケットを1つだけ作成**し、全フレームで使い回す。

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
        serialized = pickle.dumps(data)
        sock.sendto(serialized, (target_ip, self.udp_port))
    except Exception as e:
        print(f"UDP send error: {e}")
```

- UDP はコネクションレスなので、1つのソケットで任意の宛先に `sendto()` 可能
- ただし、宛先IPごとにソケットを分けることで、送信先の識別とエラーハンドリングが容易になる
- 破棄は `shutdown()` 時に一括で行う

### 4.3 データフォーマット（pickle）

受信側（Raspi）もPythonであるため、`pickle` によるシリアライズを継続使用する。

**GPS_INPUT 用データ（50Hz）**

```python
{
    "type": "gps",
    "rigid_body_id": 1,
    "pos": [x, y, z],      # NED座標 [m]
    "vel": [vx, vy, vz],   # NED速度 [m/s]
}
```

- `time_usec` は含めない（ArduPilot側で不使用のため）
- 位置は Motive のグローバル座標を NED に変換したもの

**SYSTEM_TIME 用データ（50Hz 送信、Raspi側で3Hzに間引き）**

```python
{
    "type": "system_time",
    "time_ns": 1717084800000000000,  # time.time_ns() の値
}
```

---

## 5. 送信周期の制御

### 5.1 SDK側（Motive PC）

**間引きは行わない。GPS / SYSTEM_TIME ともに 50Hz で Raspi に送信する。**

`NatNetClient.__unpack_rigid_body()` は Motive からフレームが届くたび（50Hz）に呼び出され、GPSデータとSYSTEM_TIMEデータの両方を毎回送信する。

```python
def __unpack_rigid_body(self, ...):
    # GPSデータ送信（毎フレーム = 50Hz）
    gps_data = {
        "type": "gps",
        "rigid_body_id": new_id,
        "pos": [x, y, z],
        "vel": [vx, vy, vz],
    }
    self.send_udp_data(gps_data, target_ip, new_id)
    
    # SYSTEM_TIME送信（毎フレーム = 50Hz、間引きなし）
    system_time_data = {
        "type": "system_time",
        "time_ns": time.time_ns()
    }
    self.send_udp_data(system_time_data, target_ip, new_id)
```

### 5.2 Raspi側

Raspi側で受信したデータを以下の周期で ArduPilot に転送する。

| データ | SDK→Raspi周期 | Raspi→ArduPilot周期 | 実装方針 |
|--------|--------------|---------------------|---------|
| `GPS_INPUT` | 50Hz | **50Hz** | 受信したGPSデータをそのままMAVLink送信 |
| `SYSTEM_TIME` | 50Hz | **3Hz** | 受信カウンタで間引き（5回に1回のみMAVLink送信） |

```python
# Raspi側 間引きロジック
gps_frame_count = 0

while True:
    data, addr = sock.recvfrom(4096)
    msg = pickle.loads(data)
    
    if msg["type"] == "gps":
        gps_frame_count += 1
        # GPS_INPUT は毎回送信（50Hz）
        send_mavlink_gps_input(msg["pos"], msg["vel"])
    
    elif msg["type"] == "system_time":
        # SYSTEM_TIME は GPS 5回に1回だけ送信（50/5 = 10Hz...ではなく 3Hz）
        # GPS_INPUT 送信回数を基準に間引く
        if gps_frame_count % 5 == 0:
            send_mavlink_system_time(msg["time_ns"])
```

> GPS_INPUT 50Hz / 5 = 10Hz だが、実装上は GPS_INPUT の送信回数をカウントして5回に1回送る方式。
> 必要に応じてカウンタの除数を調整可能。

---

## 6. Raspi側の処理（参考）

Raspi側の受信スクリプトは以下の流れで動作する：

1. UDP ソケットで `pickle` データを受信
2. `type` フィールドでメッセージ種別を判定
   - `"gps"` → `GPS_INPUT` MAVLinkメッセージを生成・送信
   - `"system_time"` → `SYSTEM_TIME` MAVLinkメッセージを生成・送信
3. MAVLinkメッセージを Flight Controller にシリアル経由で送信

```python
# Raspi側 擬似コード
import socket, pickle

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 15769))

while True:
    data, addr = sock.recvfrom(4096)
    msg = pickle.loads(data)
    
    if msg["type"] == "gps":
        # GPS_INPUT 送信
        send_mavlink_gps_input(msg["pos"], msg["vel"])
    
    elif msg["type"] == "system_time":
        # SYSTEM_TIME 送信
        send_mavlink_system_time(msg["time_ns"])
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
  "system_time_divider": 5
}
```

| 項目 | 説明 |
|------|------|
| `udp_targets` | 剛体ID → 送信先IPアドレスのマッピング |
| `udp_port` | UDP送信先ポート番号 |
| `system_time_divider` | Raspi側でSYSTEM_TIMEを間引く除数（デフォルト: 5。GPS 5回に1回 = 50/5 = 10Hz。用途に応じて調整） |

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

```json
{
  "udp_targets": {
    "1": "192.168.1.10",
    "2": "192.168.1.20"
  },
  "udp_port": 15769,
  "recording_enabled": true
}
```

| 項目 | 説明 |
|------|------|
| `recording_enabled` | `true` に設定すると記録機能が有効化される（`NatNetClient.__init__` で読み込み: L126-130）。デフォルトは `false` |

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
| **UDP送信**（§3, §4） | `GPS_INPUT`（NED位置・速度） + `SYSTEM_TIME`（`time.time_ns()`） | NED座標系（`motive_to_ned()` 変換済み） | ドローン制御 |

CSV 記録は `GPS_INPUT` / `SYSTEM_TIME` の UDP 送信とは独立して動作し、互いに影響しない。

---

## 10. 変更履歴

| 日付 | 変更内容 |
|------|---------|
| 2026-05-31 | Enterキー操作を3ステートトグルから2ステートトグルに簡略化（全0行マーカー廃止、Enterごとに新規CSV生成）。CSVサンプルのtimestampを time.time_ns() 整数形式に修正 |
| 2026-05-31 | CSV記録の timestamp を Motive公式タイムスタンプから SDK生成(`time.time_ns()`) に修正（Motiveがタイムスタンプを送信しない実装に合わせた設計変更） |
| 2026-05-31 | 初版作成。Motiveからのタイムスタンプ未送信に対応。`time.time_ns()` による `SYSTEM_TIME` 送信方式に変更。SDK側はGPS/SYSTEM_TIMEともに50Hzで間引きなし送信。Raspi側でSYSTEM_TIMEをGPS 5回に1回（3Hz）に間引いてArduPilot転送。`GPS_INPUT` から時刻フィールドを削除。ソケット永続化の設計を追加。SDK側ルーティング方式を正式採用。CSV記録機能セクションを追加。 |
