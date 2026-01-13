# Motive_SDK 全体README

## 概要

このリポジトリは、OptiTrack MotiveのNatNetプロトコルを用いてモーションキャプチャデータを受信し、各種座標変換やUDP送信を行うPythonサンプル群です。用途や送信仕様の異なる複数のサンプルが含まれています。

## ディレクトリ構成

- `archive/NatNet_5F50_UDP/`：基本的なNatNet受信・UDP送信サンプル
 - `NED_GPS_double/`：高精度NED-GPS変換・マルチターゲットUDP送信サンプル

各ディレクトリには以下の主要ファイルが含まれます：
- `PythonSample.py`：メイン実行スクリプト
- `NatNetClient.py`：NatNet通信・データ処理
- `DataDescriptions.py`, `MoCapData.py`：データ構造定義

## 共通の使い方

1. **依存パッケージのインストール**
   - Python 3.x が必要です。
   - 必要に応じて `pyned2lla` などを `pip install` でインストールしてください。

2. **Motiveのストリーミング設定**
   - Motive側でデータストリーミングを有効化し、IPアドレスやポートを各スクリプトの設定に合わせてください。

3. **サンプルの実行**
   - 各ディレクトリで `PythonSample.py` を実行します。
     ```sh
     python PythonSample.py
     ```
   - 必要に応じてコマンドライン引数でIPアドレスやマルチキャスト/ユニキャスト指定が可能です。

4. **データ送信先の設定**
   - 剛体IDごとに送信先IPアドレスを設定できます（スクリプト内の変数を編集）。

5. **座標変換・送信仕様**
   - 各サンプルでMotive座標→NED座標→GPS座標変換やYaw角計算、UDP送信仕様が異なります。詳細は各ディレクトリのREADMEを参照してください。

## 注意事項
- Motiveのバージョンやネットワーク設定によっては動作しない場合があります。
- UDP送信先のデバイスが正しく受信できるよう、ファイアウォールやネットワーク設定に注意してください。
- サンプルは研究・開発用途向けです。実運用時は十分なテストを行ってください。

## 参考
- [OptiTrack NatNet SDK](https://optitrack.com/)
- [pyned2lla（NED-GPS変換）](https://pypi.org/project/pyned2lla/)

---

各サンプルの詳細な違い・用途は各ディレクトリのREADMEを参照してください。
