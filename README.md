# Motive SDK - OptiTrack NatNet Python Client Collection

## 概要
このリポジトリは、OptiTrack MotiveからNatNetプロトコルを使用してリアルタイムにモーションキャプチャデータを取得し、各種処理を行うPythonプログラム集です。

## フォルダ構成

### GPS_NED_w/
**NED-GPS変換・マルチターゲットUDP送信・データ記録システム**

主な機能:
- Motive座標系 → NED座標系 → GPS座標系への変換
- クォータニオンからYaw角計算
- 剛体IDごとに異なるIPアドレスへのUDP送信（可変個数対応）
- **エンターキーによるデータ記録機能**（50Hzサンプリング）
- **CSV自動保存**（タイムスタンプ、位置、クォータニオン）
- JSON設定ファイルでの送信先・記録機能管理

設定ファイル: `config.json`
```json
{
    "udp_targets": {
        "1": "192.168.11.16",
        "2": "192.168.11.61"
    },
    "recording_enabled": true
}
```

詳細: [GPS_NED_w/README.md](GPS_NED_w/README.md)

### NatNet_Quaternion_Logger/
**剛体データロギングシステム**（存在する場合）

### archive/
過去のバージョンやテスト用プログラムのアーカイブ

## 必要環境

- Python 3.x
- 必要なライブラリ:
  ```bash
  pip install pyned2lla
  ```

## 基本的な使い方

1. 対象フォルダに移動
   ```bash
   cd GPS_NED_w
   ```

2. 設定ファイルを編集
   ```bash
   # config.json を編集してUDP送信先や記録機能を設定
   ```

3. プログラム実行
   ```bash
   python PythonSample.py
   ```

4. データ記録（記録機能が有効な場合）
   - エンターキーで記録開始
   - 再度エンターキーで記録停止→CSV保存

5. 停止
   ```bash
   Ctrl+C
   ```

## Motive設定

1. Motiveを起動し、「View」→「Data Streaming」を開く
2. 「Broadcast Frame Data」を有効化
3. 「Local Interface」に実行PCのIPアドレスを設定
4. ストリーミング周波数を確認（通常50Hz）

## 共通の依存関係

すべてのプログラムは以下のコアファイルを使用:
- `NatNetClient.py` - NatNetプロトコルクライアント
- `DataDescriptions.py` - データ記述クラス
- `MoCapData.py` - モーションキャプチャデータクラス

## トラブルシューティング

### データが受信できない
- Motiveのストリーミング設定を確認
- ファイアウォールでポート1510, 1511を開く
- ローカルIPアドレスが正しいか確認

### UDP送信エラー
- 送信先IPアドレスの到達可能性を確認
- ネットワーク接続を確認
- 送信先ポート（デフォルト15769）が開いているか確認

### 記録機能が動作しない
- config.jsonで `recording_enabled: true` になっているか確認
- ターミナルでエンターキー入力が可能か確認

## 開発情報

- OptiTrack NatNet SDK準拠
- Python 3.x対応
- UDP通信
- サンプリング周波数: 50Hz (Motiveと同期)

## ライセンス

OptiTrack NatNet SDKのライセンスに準拠します。

## 更新履歴

- 2026-01-13: GPS_NED_wにデータ記録機能を追加
- 2026-01-13: config.jsonによる統合設定管理を導入
- 2026-01-13: 可変個数剛体のUDP送信対応
