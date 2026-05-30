# Motive-ArduPilot 時刻同期システム 最終設計ドキュメント

---

## 1. システム全体アーキテクチャ

```
┌──────────────────────────────────────────────────────────────────────────┐
│                         Motive PC (Windows)                               │
│  ┌─────────────┐    ┌──────────────────┐    ┌───────────────────────┐    │
│  │ OptiTrack    │───▶│ NatNetClient.py  │───▶│ UDP送信 (struct 31B)  │    │
│  │ Motive       │    │ ・座標変換       │    │ ポート: 15769         │    │
│  │ (50Hz stream)│    │ ・時刻同期       │    │ プロトコル: struct    │    │
│  └─────────────┘    │ ・GPS変換        │    │ '<BiiiHdd'            │    │
│                      └──────────────────┘    └───────────┬───────────┘    │
└──────────────────────────────────────────────────────────┼──────────────┘
                                                           │
                                          Ethernet / Wi-Fi  │
                                          UDP 31-byte binary │
                                                           │
┌──────────────────────────────────────────────────────────┼──────────────┐
│                      Raspberry Pi 5                       ▼               │
│  ┌──────────────────┐    ┌──────────────────┐    ┌───────────────────┐   │
│  │ UDP受信          │───▶│ PeriodicSender   │───▶│ MAVLink送信       │   │
│  │ (50Hz受信)       │    │ (15Hz定期送信)   │    │ /dev/ttyAMA0      │   │
│  │ struct.unpack    │    │ ・SYSTEM_TIME    │    │ 1Mbps, RTS/CTS    │   │
│  │ '<BiiiHdd'       │    │ ・GPS_INPUT      │    │                   │   │
│  └──────────────────┘    └──────────────────┘    └─────────┬─────────┘   │
└────────────────────────────────────────────────────────────┼────────────┘
                                                             │
                                                    UART TX/RX │
                                                             │
┌────────────────────────────────────────────────────────────┼────────────┐
│                       Pixhawk 6C                            ▼             │
│  ┌──────────────────┐    ┌──────────────────┐    ┌───────────────────┐   │
│  │ MAVLink受信      │───▶│ EKF3 (推定)      │───▶│ フライト制御      │   │
│  │ ・SYSTEM_TIME    │    │ ・GPS融合         │    │                   │   │
│  │ ・GPS_INPUT      │    │ ・Observer補正    │    │                   │   │
│  └──────────────────┘    └──────────────────┘    └───────────────────┘   │
│                                                                          │
│  設定: GPS1_TYPE=14 (MAVLink GPS Input)                                  │
│        EK3_SRC1_POSXY=3 (GPS)                                            │
└──────────────────────────────────────────────────────────────────────────┘
```

### 各ノードの役割

| ノード | 役割 | キーファイル |
|--------|------|-------------|
| **Motive PC** | OptiTrackから剛体データ取得、座標変換(Motive→NED→GPS)、高精度タイムスタンプ付与、UDPバイナリ送信 | `NatNetClient.py` L946-1045 |
| **Raspberry Pi 5** | UDP受信(50Hz)、15Hz間引き+定期送信、MAVLink `SYSTEM_TIME`+`GPS_INPUT` 送信 | `send_GPS2.py` L250-347 |
| **Pixhawk 6C** | MAVLink受信、RTC設定(`SYSTEM_TIME`)、EKF3 GPS融合(`GPS_INPUT`) | GPS1_TYPE=14 |


---

## 2. 時刻同期の設計思想

### 2.1 なぜ時刻同期が必要か：「分断された時計」問題

```
【問題: 時刻同期なしの場合】

 Motive PC                     Raspberry Pi 5               Pixhawk 6C
 ═══════                       ══════════════               ══════════
                                                          
 Motiveタイマー                Python time.time()           STM32 RTC
 (ソフトウェアタイマー)         (OSクロック)                 (ハードウェアRTC)
      │                              │                          │
      │  t=100.000s                  │                          │
      │  ──────────▶ UDP送信 ──────▶ │ t=1738138400.123         │
      │                              │ ───▶ GPS_INPUT ────────▶ │ t=?????
      │                              │                          │
      ▼                              ▼                          ▼
  時刻の基準がバラバラ！ どのデータが「いつ」のものか特定不能！
```

**「分断された時計」問題**: 3つのノードがそれぞれ独立した時計を持ち、
- Motive の `timestamp` (ソフトウェアタイマー、0.000s からの相対秒)
- Raspberry Pi の `time.time()` (Unixエポック、1970-01-01 からの絶対秒)
- Pixhawk の RTC (ハードウェアRTC、起動時不定)

これらを相互に変換する仕組みが必要。

### 2.2 基準時刻方式：起動時キャリブレーション + フレーム毎補間

```
【解決策: 基準時刻方式】

  起動時（最初の1フレームのみ）:
  ┌─────────────────────────────────────────────────────────────┐
  │                                                             │
  │   Motiveタイマー                    Unixエポック            │
  │   (t=0 からの相対秒)                (1970年からの絶対秒)     │
  │        │                                  │                 │
  │   base_motive_ts ───────── 同時刻 ──── base_unix            │
  │   (例: 123.456789)              (例: 1738138223.456789)     │
  │                                                             │
  │   time.time() はこの1回だけ呼び出し                          │
  └─────────────────────────────────────────────────────────────┘

  フレーム毎（50Hz）:
  ┌─────────────────────────────────────────────────────────────┐
  │                                                             │
  │   unix_ts = base_unix + (motive_ts - base_motive_ts)        │
  │                  ↑                    ↑                     │
  │            起動時のUnix時刻      Motive高精度タイマー        │
  │            (定数オフセット)      による経過時間              │
  │                                                             │
  │   motive_ts = 124.556789  (フレームサフィックスから取得)     │
  │   → unix_ts = 1738138223.456789 + (124.556789-123.456789)   │
  │             = 1738138224.556789                              │
  └─────────────────────────────────────────────────────────────┘
```

**コード実装**（`NatNetClient.py` L959-966）:

```python
# 修正4: 基準時刻同期（初回フレームで1回だけ設定）
if self.base_motive_ts is None:
    self.base_motive_ts = timestamp          # Motiveタイマー値
    self.base_unix = time.time()             # Unix時刻（1回だけ！）
    print(f"[TimeSync] Base motive_ts={timestamp:.6f}, base_unix={self.base_unix:.6f}")

# Unix時刻を計算（Motive高精度タイマーでフレーム間隔を保証）
unix_time_sec = self.base_unix + (timestamp - self.base_motive_ts)
```



### 2.3 Motiveタイマーの高精度性

Motive の `timestamp` は NatNet フレームサフィックスに含まれる **ソフトウェアタイマー値** です。

| 特性 | Motiveタイマー | Python `time.time()` | Python `perf_counter` |
|------|---------------|---------------------|----------------------|
| 分解能 | double (float64) | float64 | float64 |
| 精度 | **サブμs** (高精度イベントタイマー) | ±1-15ms (OSジッタ) | ±1μs |
| 絶対時刻 | ❌ 相対秒のみ | ✅ Unixエポック | ❌ 任意の基準点 |
| フレーム間隔精度 | **極めて高い** | 低い (OSジッタ) | 高い |
| 使用回数 | 毎フレーム | **1回のみ** (起動時) | 不使用 |

**設計判断**: 
- `time.time()` は起動時の「絶対時刻アンカー」として**1回だけ**使用
- フレーム間の時間経過は Motiveタイマーの差分で計算（高精度）
- `perf_counter` は不要。Motiveタイマーで十分

```
time.time() のジッタ (±15ms) が入るのは base_unix の定数オフセットのみ。
フレーム間の相対時間は Motiveタイマーが保証するため、
時刻の「間隔」は高精度、「絶対値」にのみ ±15ms の定数誤差が乗る。
```

---

## 3. 発見されたバグと修正

### 3.1 timestamp=-1 バグ（修正1）

**原因**: 剛体データの処理順序が間違っていた。

```
【バグありコードのフロー（修正前）】

  __unpack_mocap_data():
  │
  ├── 1. __unpack_frame_prefix_data()    ← フレーム番号のみ
  ├── 2. __unpack_marker_set_data()
  ├── 3. __unpack_rigid_body_data()      ← ★ここで剛体をアンパック
  │       │
  │       └── for rb in rigid_body_list:
  │              process_and_send(rb)    ← ★timestamp未確定のまま送信！
  │                                        self.current_frame_timestamp = -1
  │
  ├── 4. __unpack_skeleton_data()
  ├── 5. __unpack_labeled_marker_data()
  ├── 6. __unpack_force_plate_data()

**修正後のフロー**（`NatNetClient.py` L955-1018）:

```
【修正後のコードフロー】

  __unpack_mocap_data():
  │
  ├── 1. __unpack_frame_prefix_data()
  ├── 2. __unpack_marker_set_data()
  ├── 3. __unpack_rigid_body_data()      ← 剛体データを蓄積（処理しない）
  ├── 4. __unpack_skeleton_data()
  ├── 5. __unpack_labeled_marker_data()
  ├── 6. __unpack_force_plate_data()
  ├── 7. __unpack_device_data()
  ├── 8. __unpack_frame_suffix_data()    ← ★先にtimestampを取得
  │       self.current_frame_timestamp = timestamp
  │
  └── 9. ★タイムスタンプ確定後に剛体データを一括処理
         base_motive_ts 初期化（初回のみ）
         unix_time_sec 計算
         for rb in rigid_body_list:     ← 正しいタイムスタンプ付きで処理
            GPS変換 → UDP送信
```

### 3.2 修正の要点

| 修正番号 | 内容 | ファイル:行 |
|---------|------|------------|
| 修正1 | 剛体データ蓄積→タイムスタンプ確定後→一括処理 | `NatNetClient.py` L955-1018 |
| 修正2 | UDPソケット永続化（毎回close/openしない） | `NatNetClient.py` L99-100, L228-234 |
| 修正3 | pickle→struct バイナリ化（31バイト固定長） | `NatNetClient.py` L241-251 |
| 修正4 | 基準時刻同期（base_motive_ts + base_unix） | `NatNetClient.py` L959-966 |
| 修正5 | CSVに両方のタイムスタンプを記録 | `NatNetClient.py` L330-331, L976-982 |

---

## 4. UDPプロトコル仕様

### 4.1 バイナリレイアウト（31バイト固定長）

```
Byte offset:  0        1        5        9        13       15       23       31

### 4.2 フィールド詳細

| フィールド | 型 | ビット幅 | 単位 | 意味 | 例 |
|-----------|-----|---------|------|------|-----|
| `rigid_body_id` | `uint8` | 8 | — | OptiTrack剛体ID | `1` |
| `gps_lat` | `int32` | 32 | degE7 (度×10⁷) | GPS緯度 | `360757801` (=36.0757801°) |
| `gps_lon` | `int32` | 32 | degE7 (度×10⁷) | GPS経度 | `1362132945` (=136.2132945°) |
| `gps_alt` | `int32` | 32 | mm (ミリメートル) | GPS高度 | `1234` (=1.234m) |
| `yaw_cdeg` | `uint16` | 16 | cdeg (度×100) | ヨー角 | `12345` (=123.45°) |
| `motive_timestamp` | `float64` | 64 | 秒 | Motive高精度タイマー値 | `123.456789` |
| `unix_time_sec` | `float64` | 64 | 秒 (Unix epoch) | 変換後Unix時刻 | `1738138224.557` |

### 4.3 エンディアン

リトルエンディアン (`<`)。x86/x64 (Motive PC) および ARM (Raspberry Pi) のネイティブバイトオーダー。

### 4.4 pickle → struct 改善効果

```
【改善前: pickle シリアライズ】
  サイズ: ~300-600 bytes（変動）
  形式:  Python独自形式（他言語から読めない）
  速度:  遅い（オブジェクトシリアライズ+デシリアライズ）
  安全性: pickle.loads は任意コード実行の脆弱性あり

【改善後: struct バイナリ】
  サイズ: 31 bytes（固定長）
  形式:  言語非依存のバイナリ（C/Rust/Go等からも読める）
  速度:  速い（メモリコピー+バイトオーダー変換のみ）
  安全性: 完全に安全（固定フォーマット）
```

**送信側**（`NatNetClient.py` L241-251）:
```python
serialized = struct.pack('<BiiiHdd',
                         rigid_body_id,
                         gps_lat_degE7,
                         gps_lon_degE7,
                         gps_alt_mm,
                         yaw_cdeg,
                         motive_timestamp,
                         unix_time_sec)
```

**受信側**（`send_GPS2.py` L138）:
```python
unpacked = struct.unpack('<BiiiHdd', data[:31])
rigid_body_id, lat_e7, lon_e7, alt_mm, yaw_cdeg, motive_timestamp, unix_time_sec = unpacked
```

            ┌────────┬────────┬────────┬────────┬────────┬────────┬────────┐
            │  B     │  i     │  i     │  i     │  H     │  d     │  d     │
            │uint8   │ int32  │ int32  │ int32  │uint16  │float64 │float64 │
            ├────────┼────────┼────────┼────────┼────────┼────────┼────────┤
   Value:   │ rb_id  │lat_e7  │lon_e7  │alt_mm  │yaw_cdeg│motive_ts│unix_ts │
            └────────┴────────┴────────┴────────┴────────┴────────┴────────┘
             1B       4B       4B       4B       2B       8B       8B

   struct format: '<BiiiHdd'  (リトルエンディアン)
   合計: 1 + 4 + 4 + 4 + 2 + 8 + 8 = 31 bytes
```

---

## 5. ArduPilotの時刻処理（調査結果）

### 5.1 GPS_INPUT.time_usec は完全無視される

```
GPS_INPUT メッセージ:
┌──────────────────────────────────────────────────────────────┐
│ time_usec        ← ★EKFはまったく参照しない！                │
│ gps_id           ← GPSインスタンスID                         │
│ ignore_flags     ← ビットマスク                              │
│ time_week_ms     ← GPS週+ミリ秒（代わりにこれを使う）        │
│ time_week        ←                                            │
│ fix_type         ← 3=3D Fix                                  │
│ lat, lon, alt    ← 緯度・経度・高度                          │
│ ...                                                          │
└──────────────────────────────────────────────────────────────┘

EKF内部では time_usec が参照されるコードパスが存在しないことが
ArduPilotソースコード調査で確認された。
```

### 5.2 SYSTEM_TIME が RTC を設定する

```
SYSTEM_TIME メッセージ:
┌──────────────────────────────────────────────────────────────┐
│ time_unix_usec   ← Unixエポック（μs単位）                    │
│ time_boot_ms     ← ブートからの経過時間（ms）                │
└──────────────────────────────────────────────────────────────┘

ArduPilot内部処理:
  SYSTEM_TIME受信
       │
       ▼
  AP_RTC.set_utc_usec(time_unix_usec)
       │
       ▼
  RTC.epoch_us = time_unix_usec / 1_000_000  ← Unix時刻（秒）
  RTC.TimeUS   = AP_HAL::micros()            ← その時のブート時間（μs）
```

**送信コード**（`send_GPS2.py` L93-108）:
```python
def send_system_time(self, unix_time_sec):
    time_unix_usec = int(unix_time_sec * 1_000_000)
    self.master.mav.system_time_send(
        time_unix_usec,   # Motive基準のUnix時刻
        0                 # time_boot_ms
    )
```

### 5.3 RTCログとGPSログの突合方法

```
【RTC と GPS の関係図】

  SYSTEM_TIME到着時刻
       │
       ├── RTC.epoch_us = unix_time_sec (秒単位のUnix時刻)
       └── RTC.TimeUS   = AP_HAL::micros() (その時のブート時間)

  GPSデータ到着時:
       GPS.TimeUS はブート時間（AP_HAL::micros()）で記録される

  GPSの絶対時刻を求めるには:
       GPS絶対時刻(Unix秒) = RTC.epoch_us + (GPS.TimeUS - RTC.TimeUS)
                              \___________/   \________________________/
                               基準時刻        RTC設定時からの経過時間
```

**Raspberry Pi側の送信順序**（`send_GPS2.py` L221-224）:
```python
# 最初にSYSTEM_TIME送信（ArduPilotのRTCをMotive Unix時刻に設定）
self.ardupilot.send_system_time(self.latest_data['unix_time_sec'])
# 次にGPS_INPUT送信
success = self.ardupilot.send_gps_input(...)
```

SYSTEM_TIMEをGPS_INPUTの**直前に**送信することで、RTCが最新のUnix時刻で更新された状態でGPSデータが到着する。


  ├── 7. __unpack_device_data()
  └── 8. __unpack_frame_suffix_data()    ← ★ここでやっとtimestamp取得
          self.current_frame_timestamp = timestamp  ← 手遅れ！

---

## 6. データフロー詳細

### 6.1 Motive PC側の処理パイプライン

```
  Motive 50Hz データ到着
       │
       ▼
  ┌─────────────────────────────────────────────────────────┐
  │ 1. NatNet パケット受信 (data_socket, port 1511)         │
  │    __data_thread_function()                             │
  └──────────────────────────┬──────────────────────────────┘
                             │
                             ▼
  ┌─────────────────────────────────────────────────────────┐
  │ 2. データアンパック                                      │
  │    __unpack_mocap_data():                               │
  │    ├── Frame Prefix (frame_number)                      │
  │    ├── MarkerSet Data                                   │
  │    ├── Rigid Body Data ← 蓄積のみ（処理しない）          │
  │    ├── Skeleton Data                                    │
  │    ├── Labeled Marker Data                              │
  │    ├── Force Plate Data                                 │
  │    ├── Device Data                                      │
  │    └── Frame Suffix Data ← timestamp ここで取得         │
  └──────────────────────────┬──────────────────────────────┘
                             │
                    timestamp 確定
                    base_motive_ts / base_unix 初期化（初回のみ）
                    unix_time_sec = base_unix + (ts - base_motive_ts)
                             │
                             ▼
  ┌─────────────────────────────────────────────────────────┐
  │ 3. 剛体ごとの処理ループ                                  │
  │    for rb in rigid_body_data.rigid_body_list:           │
  │    ┌─────────────────────────────────────────────────┐  │
  │    │ a. Motive座標系 → NED座標系 変換                  │  │
  │    │    motive_to_ned()   [L165-180]                  │  │
  │    │    X(北)→N, Z(東)→E, Y(上)→D(符号反転)           │  │
  │    ├─────────────────────────────────────────────────┤  │
  │    │ b. NED座標系 → GPS座標系 変換                     │  │
  │    │    ned_to_gps()   [L182-204]                     │  │
  │    │    pyned2lla.ned2lla() を使用                     │  │
  │    ├─────────────────────────────────────────────────┤  │
  │    │ c. クォータニオン → Yaw角                         │  │
  │    │    quaternion_to_yaw_degrees()  [L206-226]       │  │
  │    ├─────────────────────────────────────────────────┤  │
  │    │ d. 整数表現に変換                                  │  │
  │    │    lat_e7 = int(round(lat * 1e7))               │  │
  │    │    lon_e7 = int(round(lon * 1e7))               │  │
  │    │    alt_mm = int(round(alt * 1000))              │  │
  │    │    yaw_cdeg = int(round(yaw * 100))             │  │
  │    ├─────────────────────────────────────────────────┤  │
  │    │ e. struct バイナリにパック                          │  │
  │    │    struct.pack('<BiiiHdd', ...) → 31 bytes     │  │
  │    ├─────────────────────────────────────────────────┤  │
  │    │ f. UDP送信                                        │  │
  │    │    sock.sendto(data, (target_ip, 15769))        │  │
  │    └─────────────────────────────────────────────────┘  │
  └─────────────────────────────────────────────────────────┘
```

```


### 6.2 Raspberry Pi側の処理パイプライン

```
  UDPデータ受信 (50Hz)
       │
       ▼
  ┌─────────────────────────────────────────────────────────┐
  │ 1. UDP受信 (ノンブロッキング)                             │
  │    struct.unpack('<BiiiHdd', data[:31])                │
  │    → (rb_id, lat_e7, lon_e7, alt_mm, yaw_cdeg,          │
  │       motive_ts, unix_ts)                               │
  └──────────────────────────┬──────────────────────────────┘
                             │
                    latest_data 更新（スレッドセーフ）
                             │
                             ▼
  ┌─────────────────────────────────────────────────────────┐
  │ 2. 定期送信ループ (15Hz, 66.67ms間隔)                    │
  │    PeriodicSender._send_loop():                         │
  │                                                         │
  │    高精度タイミング制御:                                  │
  │      next_send_time += interval (66.67ms)               │
  │      time.sleep(sleep_time - 0.001)                    │
  │      while time.time() < next_send_time: pass (busy)   │
  │                                                         │
  │    送信順序（重要！）:                                    │
  │      a. SYSTEM_TIME 送信                                │
  │         → PixhawkのRTCをMotive Unix時刻に設定            │
  │      b. GPS_INPUT 送信                                  │
  │         → 位置・高度・Yaw角をEKFに注入                   │
  └─────────────────────────────────────────────────────────┘
```

### 6.3 CSV記録形式

Motive PC側で記録（`NatNetClient.py` L329-335, L975-982）:

```
CSVヘッダー:
  motive_timestamp, unix_time_sec, rigid_body_id,
  pos_x, pos_y, pos_z,
  qx, qy, qz, qw

サンプル行:
  123.456789, 1738138224.556789, 1,
  1.234, 2.345, 0.123,
  0.000, 0.000, 0.707, 0.707
```

| カラム | 説明 | 単位 |
|--------|------|------|
| `motive_timestamp` | Motive高精度タイマー値 | 秒 |
| `unix_time_sec` | 変換後Unix時刻 | 秒 |
| `rigid_body_id` | 剛体ID | — |
| `pos_x, pos_y, pos_z` | Motive生座標 | m |
| `qx, qy, qz, qw` | Motive生クォータニオン | — |

### 6.4 複数剛体IDの扱い

```
config.json:
{
    "udp_targets": {
        "1": "192.168.11.16",    ← RB#1 → Pixhawk #1
        "2": "192.168.11.61"     ← RB#2 → Pixhawk #2
    }

---

## 7. 精度の考察

### 7.1 誤差モデル

```
                    ┌─────────────────────────────┐
                    │  真のUnix時刻                │
                    │  (原子時計などの理想値)       │
                    └─────────────┬───────────────┘
                                  │
                    ┌─────────────┼───────────────┐
                    │             │               │
                    ▼             ▼               ▼
           ┌──────────┐  ┌──────────────┐  ┌──────────────┐
           │ 絶対誤差  │  │ 相対誤差     │  │ ジッタ      │
           │ ±15ms    │  │ サブμs       │  │ 0            │
           │ (定数)   │  │ (Motive精度) │  │ (Motive精度) │
           └──────────┘  └──────────────┘  └──────────────┘
```

### 7.2 誤差の内訳

| 誤差要因 | 種類 | 大きさ | 説明 |
|---------|------|--------|------|
| `time.time()` のOSジッタ | 絶対オフセット（定数） | **±15ms** | 起動時の1回のみ発生。全フレームに等しく加算 |
| Motiveタイマー精度 | 相対精度 | **サブμs** | フレーム間隔の計測に使用。極めて高精度 |
| UDPネットワーク遅延 | 絶対オフセット | 0.1-1ms | Ethernetの場合は極小 |
| struct パック/アンパック | 変換誤差 | **0** | 整数変換による丸め誤差のみ（degE7, mm, cdeg） |

### 7.3 絶対精度（±15msの定数オフセット）

```
unix_ts = base_unix + (motive_ts - base_motive_ts)
          \______/
          ±15ms の誤差を含む（time.time()のジッタ）

この誤差は base_unix にのみ含まれ、全フレームで一定。
→ ログ解析時に 15ms の定数バイアスとして扱える
→ 制御周期（50Hz=20ms）に対しては許容範囲
```

### 7.4 相対精度（Motiveタイマーによるサブμs）

```
フレーム間隔の精度:
  Δt = motive_ts[n] - motive_ts[n-1]
  
  期待値: 20.000ms (50Hz)
  ジッタ: サブμsオーダー

  これは Motive の高精度ソフトウェアタイマーが保証する。
  Windows の QueryPerformanceCounter ベース。
```

### 7.5 Pixhawk EKFへの影響

```
EKF3 のタイムスタンプ要件:
  - GPSデータの「時刻」は EKF の prediction step との同期に使用
  - GPS_INPUT.time_usec ではなく RTC/GPS.TimeUS が使われる
  - 15ms の定数オフセットは EKF の innovation に影響しない
    （相対的なフレーム間隔が正しいため）

  EKF融合パイプライン:
  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐
  │IMU予測   │───▶│GPS更新   │───▶│姿勢出力  │───▶│制御出力  │

---

## 8. ファイル構成と変更内容

### 8.1 ファイル一覧

```
Motive_SDK/GPS_NED_w_timestamp/     ← Motive PC側（本ディレクトリ）
├── DESIGN.md                       ★ このドキュメント（新規）
├── README.md                       概要説明
├── config.json                     UDP送信先設定
├── NatNetClient.py                 ★ メインクライアント（全修正を含む）
├── PythonSample.py                 実行エントリポイント
├── DataDescriptions.py             NatNetデータ記述クラス
├── MoCapData.py                    モーキャプデータクラス
├── PythonClient.pyproj             VSプロジェクトファイル
└── PythonClient.sln                VSソリューションファイル

Mavlink_raspi/EKF/                  ← Raspberry Pi 5側
└── send_GPS2.py                    ★ UDP受信+MAVLink送信ブリッジ
```

### 8.2 削除されたもの

| 削除項目 | 理由 | 代替 |
|---------|------|------|
| `pickle` シリアライズ | 可変長・低速・Python依存・脆弱性 | `struct.pack('<BiiiHdd')` (31B固定) |
| `data_buffer` / `recording_data` の複雑な管理 | バグの温床 | シンプルなリスト蓄積 |
| 重複フィールド (`data_no`, `frame_time`, `motive_position`, `ned_position` 等) | UDP帯域の無駄 | 必要最低限の6フィールド |
| `time.perf_counter` | 不要（Motiveタイマーで十分） | Motiveタイマー差分で高精度に計算 |

### 8.3 新規追加されたもの

| 追加項目 | 場所 | 説明 |
|---------|------|------|
| `SYSTEM_TIME` 送信 | `send_GPS2.py` L93-108 | ArduPilotのRTC設定 |
| `base_motive_ts` / `base_unix` | `NatNetClient.py` L103-104 | 基準時刻同期用 |
| `struct` バイナリプロトコル | `NatNetClient.py` L241-251 | 31バイト固定長フォーマット |
| 永続UDPソケット | `NatNetClient.py` L99-100, L228-234 | フレーム毎のconnect/disconnect回避 |
| `unix_time_sec` フィールド | `NatNetClient.py` L966, `send_GPS2.py` L294 | 変換後Unix時刻 |
| CSV両タイムスタンプ | `NatNetClient.py` L330-331 | motive_ts + unix_ts の両方を保存 |

### 8.4 変更のインパクトまとめ

```
【変更前】                               【変更後】
                                        
 Motive PC                               Motive PC
   pickle 可変長 (~300B)                   struct 31B 固定長
   timestamp=-1 バグ                       timestamp 正しく伝播
   時刻同期なし                             基準時刻方式で高精度同期
   ↓                                      ↓
 Raspberry Pi                            Raspberry Pi
   pickle 受信                             struct unpack (高速)
   GPS_INPUT のみ                          SYSTEM_TIME + GPS_INPUT
   不安定なタイミング                       15Hz 高精度定期送信
   ↓                                      ↓
 Pixhawk                                 Pixhawk
   time_usec が不定                        RTC = Unix時刻（SYSTEM_TIME）
   GPS時刻が不正確                         GPS時刻が正確に突合可能
```

---

## 付録A: 用語集

| 用語 | 説明 |
|------|------|
| **Motive** | OptiTrack社のモーションキャプチャソフトウェア |
| **NatNet** | Motiveが使用するUDPストリーミングプロトコル |
| **NED** | North-East-Down 座標系（航空機の標準座標系） |
| **degE7** | 度×10⁷。例: 36.0757801° = 360757801 |
| **cdeg** | センチ度（度×100）。例: 123.45° = 12345 |
| **RTC** | Real-Time Clock。Pixhawkのハードウェア時計 |
| **EKF** | Extended Kalman Filter（拡張カルマンフィルタ） |
| **MAVLink** | ドローン通信プロトコル |
| **RTS/CTS** | ハードウェアフロー制御（UART） |

## 付録B: クイックスタート

```bash
# Motive PC側
cd /home/taki/Motive_SDK/GPS_NED_w_timestamp/
python PythonSample.py

# Raspberry Pi側
cd /home/taki/Mavlink_raspi/EKF/
python3 send_GPS2.py
```

## 付録C: 参考資料

- ArduPilot MAVLink GPS_INPUT: https://mavlink.io/en/messages/common.html#GPS_INPUT
- ArduPilot MAVLink SYSTEM_TIME: https://mavlink.io/en/messages/common.html#SYSTEM_TIME
- OptiTrack NatNet SDK: https://optitrack.com/support/downloads/developer-tools.html

