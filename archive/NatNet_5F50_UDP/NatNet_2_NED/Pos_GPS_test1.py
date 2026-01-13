import pyned2lla
import math
import time

# 角度変換用の定数
D2R = math.pi / 180.0  # 度をラジアンに変換
R2D = 180.0 / math.pi  # ラジアンを度に変換

# WGS84楕円体を定義
wgs84 = pyned2lla.wgs84()

# 基準点のGPS座標（例：日本の座標）
lat0, lon0, alt0 = 36.075780, 136.213290, 0.0  # 緯度、経度、高度

steart = time.time()

for i in range(1000):
    # MotiveからのNED座標（メートル単位）
    north, east, down = 10, 5, -2  # North, East, Down

    # NED座標をGPS座標に変換
    lat, lon, alt = pyned2lla.ned2lla(
        lat0 * D2R,    # 基準緯度（ラジアン）
        lon0 * D2R,    # 基準経度（ラジアン）
        alt0,          # 基準高度（メートル）
        north,         # 北方向距離（メートル）
        east,          # 東方向距離（メートル）
        down,          # 下方向距離（メートル）
        wgs84          # 楕円体モデル
    )

    # 結果を度単位に変換
    lat_deg = lat * R2D
    lon_deg = lon * R2D

print(time.time() - steart)

print(f"変換結果: 緯度={lat_deg:.8f}°, 経度={lon_deg:.8f}°, 高度={alt:.2f}m")
