import sys
import socket
import struct
from threading import Thread
import copy
import time
import DataDescriptions
import MoCapData
import math
import pyned2lla
import pickle
import json
import os

def trace( *args ):
    # uncomment the one you want to use
    #print( "".join(map(str,args)) )
    pass

#Used for Data Description functions
def trace_dd( *args ):
    # uncomment the one you want to use
    #print( "".join(map(str,args)) )
    pass

#Used for MoCap Frame Data functions
def trace_mf( *args ):
    # uncomment the one you want to use
    #print( "".join(map(str,args)) )
    pass

def get_message_id(data):
    message_id = int.from_bytes( data[0:2], byteorder='little', signed=True )
    return message_id

# Create structs for reading various object types to speed up parsing.
Vector2 = struct.Struct( '<ff' )
Vector3 = struct.Struct( '<fff' )
Quaternion = struct.Struct( '<ffff' )
FloatValue = struct.Struct( '<f' )
DoubleValue = struct.Struct( '<d' )

# Increased size for the newest force plate
FPCalMatrixRow = struct.Struct( '<ffffffffffff' )
FPCorners = struct.Struct( '<ffffffffffff' )

class NatNetClient:
    # default to > 1 on / print every nth mocap frame
    print_level = 20

    def __init__( self ):
        # Change this value to the IP address of the NatNet server.
        self.server_ip_address = "127.0.0.1"

        # Change this value to the IP address of your local network interface
        self.local_ip_address = "127.0.0.1"

        # This should match the multicast address listed in Motive's streaming settings.
        self.multicast_address = "239.255.42.99"

        # NatNet Command channel
        self.command_port = 1510

        # NatNet Data channel
        self.data_port = 1511

        self.use_multicast = True

        # Set this to a callback method of your choice to receive per-rigid-body data at each frame.
        self.rigid_body_listener = None
        self.new_frame_listener = None

        # Set Application Name
        self.__application_name = "Not Set"

        # NatNet stream version server is capable of. This will be updated during initialization only.
        self.__nat_net_stream_version_server = [0,0,0,0]

        # NatNet stream version. This will be updated to the actual version the server is using during runtime.
        self.__nat_net_requested_version = [0,0,0,0]

        # server stream version. This will be updated to the actual version the server is using during initialization.
        self.__server_version = [0,0,0,0]

        # Lock values once run is called
        self.__is_locked = False

        # Server has the ability to change bitstream version
        self.__can_change_bitstream_version = False

        self.command_thread = None
        self.data_thread = None
        self.command_socket = None
        self.data_socket = None
        self.stop_threads=False

        # データバッファとカウンタ
        self.data_buffer = {}
        self.data_No = 0
        self.time_log = 0
        
        # **記録機能用変数**
        self.is_recording = False
        self.recording_data = []  # 記録データバッファ
        self.recording_start_time = None  # 記録開始時刻

        # **GPS変換用の設定**
        self.D2R = math.pi / 180.0
        self.R2D = 180.0 / math.pi
        self.wgs84 = pyned2lla.wgs84()
        
        # **基準GPS座標（7桁精度）**
        self.ref_lat = 36.0757800  # 緯度（7桁精度）
        self.ref_lon = 136.2132900  # 経度（7桁精度） 
        self.ref_alt = 0.000  # 高度（3桁精度）
        
        # **設定ファイル読み込み（config.json）**
        config_path = os.path.join(os.path.dirname(__file__), "config.json")
        try:
            with open(config_path, "r", encoding="utf-8") as f:
                config = json.load(f)
            # UDP送信先設定
            udp_targets_dict = config.get("udp_targets", {})
            self.udp_targets = {int(k): v for k, v in udp_targets_dict.items()}
            # 記録機能の有効/無効
            self.recording_enabled = config.get("recording_enabled", False)
        except Exception as e:
            print(f"[警告] config.jsonの読み込みに失敗: {e}")
            self.udp_targets = {}
            self.recording_enabled = False
        self.udp_port = 15769
        
        # UDP統計情報
        self.udp_send_count = 0
        self.udp_error_count = 0

        print(f"GPS reference initialized: ({self.ref_lat:.7f}, {self.ref_lon:.7f}, {self.ref_alt:.3f})")
        print("UDP targets configured:")
        for rb_id, ip in self.udp_targets.items():
            print(f"  Rigid Body {rb_id} → {ip}:{self.udp_port}")
        print("UDP sending at 50Hz (every frame), Console display every 50 frames")

        # Client/server message ids
        self.NAT_CONNECT = 0
        self.NAT_SERVERINFO = 1
        self.NAT_REQUEST = 2
        self.NAT_RESPONSE = 3
        self.NAT_REQUEST_MODELDEF = 4
        self.NAT_MODELDEF = 5
        self.NAT_REQUEST_FRAMEOFDATA = 6
        self.NAT_FRAMEOFDATA = 7
        self.NAT_MESSAGESTRING = 8
        self.NAT_DISCONNECT = 9
        self.NAT_KEEPALIVE = 10
        self.NAT_UNRECOGNIZED_REQUEST = 100
        self.NAT_UNDEFINED = 999999.9999

    def motive_to_ned(self, motive_x, motive_y, motive_z, motive_qx, motive_qy, motive_qz, motive_qw):
        """
        Motive座標系（左手系：X=北,Y=上,Z=東）からNED座標系（右手系：X=北,Y=東,Z=下）への変換
        """
        # 位置変換
        ned_x = motive_x   # 北→北
        ned_y = motive_z   # 東→東
        ned_z = -motive_y  # 上→下（符号反転）
        
        # クオータニオン変換（順序に注意）
        ned_qw = motive_qw   # w成分
        ned_qx = motive_qx   # X軸（北）→X軸（北）
        ned_qy = motive_qz   # Z軸（東）→Y軸（東）
        ned_qz = -motive_qy  # Y軸（上）→Z軸（下）、符号反転
        
        return ned_x, ned_y, ned_z, ned_qw, ned_qx, ned_qy, ned_qz

    def ned_to_gps(self, ned_x, ned_y, ned_z):
        """
        NED座標をGPS座標に変換（MAVLink GPS_INPUT仕様に合わせて7桁精度）
        """
        try:
            lat_rad, lon_rad, alt = pyned2lla.ned2lla(
                self.ref_lat * self.D2R,
                self.ref_lon * self.D2R, 
                self.ref_alt,
                ned_x, ned_y, ned_z,
                self.wgs84
            )
            
            # MAVLink GPS_INPUT仕様に合わせて7桁精度に制限
            lat_deg = round(lat_rad * self.R2D, 7)
            lon_deg = round(lon_rad * self.R2D, 7) 
            alt_m = round(alt, 3)  # 高度は3桁精度
            
            return lat_deg, lon_deg, alt_m
            
        except Exception as e:
            print(f"GPS conversion error: {e}")
            return None, None, None

    def quaternion_to_yaw_degrees(self, ned_quat):
        """
        クオータニオンからYaw角を計算（度単位）
        """
        try:
            ned_qw, ned_qx, ned_qy, ned_qz = ned_quat
            
            # クオータニオンからYaw角を計算
            yaw_rad = math.atan2(2.0 * (ned_qw * ned_qz + ned_qx * ned_qy),
                                1.0 - 2.0 * (ned_qy**2 + ned_qz**2))
            
            # ラジアンから度に変換し、0-360度に正規化
            yaw_deg = math.degrees(yaw_rad)
            if yaw_deg < 0:
                yaw_deg += 360.0
                
            return round(yaw_deg, 3)  # 3桁精度
            
        except Exception as e:
            print(f"Yaw calculation error: {e}")
            return 0.0

    def send_udp_data(self, rigid_body_data, target_ip, rigid_body_id):
        """UDP送信"""
        try:
            # ソケット作成
            client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            client.settimeout(2.0)  # 2秒タイムアウト
            
            # データシリアライズ
            serialized_data = pickle.dumps(rigid_body_data)
            data_size = len(serialized_data)
            
            # UDP送信実行
            bytes_sent = client.sendto(serialized_data, (target_ip, self.udp_port))
            client.close()
            
            # 送信成功
            self.udp_send_count += 1
            return True
            
        except socket.timeout:
            self.udp_error_count += 1
            print(f"✗ UDP timeout to {target_ip}:{self.udp_port} for RB{rigid_body_id}")
            return False
            
        except socket.gaierror as e:
            self.udp_error_count += 1
            print(f"✗ UDP address error to {target_ip}:{self.udp_port} for RB{rigid_body_id}: {e}")
            return False
            
        except ConnectionRefusedError:
            self.udp_error_count += 1
            print(f"✗ UDP connection refused by {target_ip}:{self.udp_port} for RB{rigid_body_id}")
            return False
            
        except Exception as e:
            self.udp_error_count += 1
            print(f"✗ UDP send error to {target_ip}:{self.udp_port} for RB{rigid_body_id}: {e}")
            return False
        
        finally:
            try:
                client.close()
            except:
                pass

    def start_recording(self):
        """記録開始"""
        if not self.recording_enabled:
            print("記録機能が無効です（config.jsonで有効化してください）")
            return
        
        if self.is_recording:
            print("すでに記録中です")
            return
            
        self.is_recording = True
        self.recording_data = []
        self.recording_start_time = time.time()
        print("==================")
        print("記録開始！")
        print("==================")

    def stop_recording(self):
        """記録停止とCSV保存"""
        if not self.is_recording:
            print("記録していません")
            return
            
        self.is_recording = False
        print("==================")
        print("記録停止！")
        print("==================")
        
        # CSV保存
        if len(self.recording_data) == 0:
            print("記録データがありません")
            return
            
        # ファイル名生成（記録開始時刻を使用）
        from datetime import datetime
        start_dt = datetime.fromtimestamp(self.recording_start_time)
        filename = start_dt.strftime("record_%Y%m%d_%H%M%S.csv")
        filepath = os.path.join(os.path.dirname(__file__), filename)
        
        # CSV書き込み
        try:
            import csv
            with open(filepath, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                # ヘッダー
                writer.writerow(['timestamp', 'rigid_body_id', 
                               'pos_x', 'pos_y', 'pos_z',
                               'quat_x', 'quat_y', 'quat_z', 'quat_w'])
                # データ
                for row in self.recording_data:
                    writer.writerow(row)
            
            print(f"CSV保存完了: {filename} ({len(self.recording_data)}行)")
        except Exception as e:
            print(f"CSV保存エラー: {e}")

    def keyboard_listener(self):
        """エンターキー監視スレッド"""
        if not self.recording_enabled:
            return
            
        print("\n[記録機能] エンターキーで記録開始/停止")
        while not self.stop_threads:
            try:
                key = input()
                if key == "":  # エンターキー
                    if not self.is_recording:
                        self.start_recording()
                    else:
                        self.stop_recording()
            except:
                break

    def set_client_address(self, local_ip_address):
        if not self.__is_locked:
            self.local_ip_address = local_ip_address

    def get_client_address(self):
        return self.local_ip_address

    def set_server_address(self,server_ip_address):
        if not self.__is_locked:
            self.server_ip_address = server_ip_address

    def get_server_address(self):
        return self.server_ip_address

    def set_use_multicast(self, use_multicast):
        if not self.__is_locked:
            self.use_multicast = use_multicast

    def can_change_bitstream_version(self):
        return self.__can_change_bitstream_version

    def set_nat_net_version(self, major, minor):
        return_code = -1
        if self.__can_change_bitstream_version and \
                ((major != self.__nat_net_requested_version[0]) or\
                 (minor != self.__nat_net_requested_version[1])):
            sz_command = "Bitstream,%1.1d.%1.1d"%(major, minor)
            return_code = self.send_command(sz_command)
            if return_code >=0:
                self.__nat_net_requested_version[0] = major
                self.__nat_net_requested_version[1] = minor
                self.__nat_net_requested_version[2] = 0
                self.__nat_net_requested_version[3] = 0
                print("changing bitstream MAIN")
                self.send_command("TimelinePlay")
                time.sleep(0.1)
                tmpCommands=["TimelinePlay",
                             "TimelineStop",
                             "SetPlaybackCurrentFrame,0",
                             "TimelineStop"]
                self.send_commands(tmpCommands,False)
                time.sleep(2)
            else:
                print("Bitstream change request failed")
        return return_code

    def get_major(self):
        return self.__nat_net_requested_version[0]

    def get_minor(self):
        return self.__nat_net_requested_version[1]

    def set_print_level(self, print_level=0):
        if(print_level >=0):
            self.print_level = print_level
        return self.print_level

    def get_print_level(self):
        return self.print_level

    def connected(self):
        ret_value = True
        if self.command_socket == None:
            ret_value = False
        elif self.data_socket ==None:
            ret_value = False
        elif self.get_application_name() == "Not Set":
            ret_value = False
        elif (self.__server_version[0] == 0) and\
                (self.__server_version[1] == 0) and\
                (self.__server_version[2] == 0) and\
                (self.__server_version[3] == 0):
            ret_value = False
        return ret_value

    # Create a command socket to attach to the NatNet stream
    def __create_command_socket( self ):
        result = None
        if self.use_multicast :
            # Multicast case
            result = socket.socket( socket.AF_INET, socket.SOCK_DGRAM, 0 )
            result.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                result.bind( ('', 0) )
            except socket.error as msg:
                print(f"ERROR: command socket error occurred: {msg}")
                print("Check Motive/Server mode requested mode agreement. You requested Multicast ")
                result = None
            except socket.herror:
                print("ERROR: command socket herror occurred")
                result = None
            except socket.gaierror:
                print("ERROR: command socket gaierror occurred")
                result = None
            except socket.timeout:
                print("ERROR: command socket timeout occurred. Server not responding")
                result = None

            if result:
                result.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                result.settimeout(2.0)
        else:
            # Unicast case
            result = socket.socket( socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            try:
                result.bind( (self.local_ip_address, 0) )
            except socket.error as msg:
                print(f"ERROR: command socket error occurred: {msg}")
                print("Check Motive/Server mode requested mode agreement. You requested Unicast ")
                result = None
            except socket.herror:
                print("ERROR: command socket herror occurred")
                result = None
            except socket.gaierror:
                print("ERROR: command socket gaierror occurred")
                result = None
            except socket.timeout:
                print("ERROR: command socket timeout occurred. Server not responding")
                result = None

            if result:
                result.settimeout(2.0)
                result.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        return result

    # Create a data socket to attach to the NatNet stream
    def __create_data_socket( self, port ):
        result = None
        if self.use_multicast:
            # Multicast case
            result = socket.socket( socket.AF_INET, socket.SOCK_DGRAM, 0)
            result.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            result.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, 
                             socket.inet_aton(self.multicast_address) + socket.inet_aton(self.local_ip_address))
            try:
                result.bind( (self.local_ip_address, port) )
            except socket.error as msg:
                print(f"ERROR: data socket error occurred: {msg}")
                print(" Check Motive/Server mode requested mode agreement. You requested Multicast ")
                result = None
            except socket.herror:
                print("ERROR: data socket herror occurred")
                result = None
            except socket.gaierror:
                print("ERROR: data socket gaierror occurred")
                result = None
            except socket.timeout:
                print("ERROR: data socket timeout occurred. Server not responding")
                result = None
        else:
            # Unicast case
            result = socket.socket( socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            result.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                result.bind( ('', 0) )
            except socket.error as msg:
                print(f"ERROR: data socket error occurred: {msg}")
                print("Check Motive/Server mode requested mode agreement. You requested Unicast ")
                result = None
            except socket.herror:
                print("ERROR: data socket herror occurred")
                result = None
            except socket.gaierror:
                print("ERROR: data socket gaierror occurred")
                result = None
            except socket.timeout:
                print("ERROR: data socket timeout occurred. Server not responding")
                result = None

            if result and (self.multicast_address != "255.255.255.255"):
                result.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, 
                                 socket.inet_aton(self.multicast_address) + socket.inet_aton(self.local_ip_address))

        return result

    # Unpack a rigid body object from a data packet
    def __unpack_rigid_body( self, data, major, minor, rb_num):
        offset = 0

        # バッファサイズチェック付きでIDを取得
        if len(data) < offset + 4:
            return offset, None

        new_id = int.from_bytes( data[offset:offset+4], byteorder='little', signed=True )
        offset += 4

        trace_mf( "RB: %3.1d ID: %3.1d"% (rb_num, new_id))

        # 位置データのバッファサイズチェック
        if len(data) < offset + 12:
            return offset, None

        pos = Vector3.unpack( data[offset:offset+12] )
        offset += 12

        trace_mf( "\tPosition : [%3.2f, %3.2f, %3.2f]"% (pos[0], pos[1], pos[2] ))

        # 姿勢データのバッファサイズチェック
        if len(data) < offset + 16:
            return offset, None

        rot = Quaternion.unpack( data[offset:offset+16] )
        offset += 16

        trace_mf( "\tOrientation : [%3.2f, %3.2f, %3.2f, %3.2f]"% (rot[0], rot[1], rot[2], rot[3] ))

        # タイムスタンプ計測
        now_time = time.perf_counter()
        data_time = now_time - self.time_log

        # **記録機能: Motiveから受信した生データを記録**
        if self.is_recording:
            # timestamp, rigid_body_id, pos_x, pos_y, pos_z, quat_x, quat_y, quat_z, quat_w
            self.recording_data.append([
                data_time,
                new_id,
                pos[0], pos[1], pos[2],
                rot[0], rot[1], rot[2], rot[3]
            ])

        # udp_targets.jsonに設定された剛体IDのデータを処理
        if new_id in self.udp_targets:
            if new_id == 1:
                self.time_log = now_time

            # Motive座標系 → NED座標系への変換
            rel_x, rel_y, rel_z = pos
            motive_qx, motive_qy, motive_qz, motive_qw = rot
            
            ned_x, ned_y, ned_z, ned_qw, ned_qx, ned_qy, ned_qz = self.motive_to_ned(
                rel_x, rel_y, rel_z,
                motive_qx, motive_qy, motive_qz, motive_qw
            )

            # NED → GPS（緯度・経度・高度）の変換
            gps_lat, gps_lon, gps_alt = self.ned_to_gps(ned_x, ned_y, ned_z)
            
            # Yaw角の計算
            yaw_deg = self.quaternion_to_yaw_degrees([ned_qw, ned_qx, ned_qy, ned_qz])

            # GPS変換が成功した場合
            if gps_lat is not None:
                # GPSデータの構築
                gps_data = {
                    'status': 'SUCCESS',
                    'id': new_id,
                    'latitude': gps_lat,
                    'longitude': gps_lon,
                    'altitude': gps_alt,
                    'yaw_degrees': yaw_deg,
                    'ned_position': [ned_x, ned_y, ned_z],
                    'ned_rotation': [ned_qw, ned_qx, ned_qy, ned_qz],
                    'motive_position': [rel_x, rel_y, rel_z],
                    'motive_rotation': [motive_qx, motive_qy, motive_qz, motive_qw],
                    'data_no': self.data_No,
                    'timestamp': data_time,
                    'frame_time': now_time
                }
                
                # 50回に1回のみコンソール表示
                if self.data_No % 50 == 0:
                    print(f"[Frame {self.data_No}] Sending ID: {new_id}, GPS: ({gps_lat:.7f}, {gps_lon:.7f}, {gps_alt:.3f}), Local Pos: ({rel_x:.3f}, {rel_y:.3f}, {rel_z:.3f})")
                
            else:
                # GPS変換失敗時のデータ
                gps_data = {
                    'status': 'GPS_CONVERSION_FAILED',
                    'id': new_id,
                    'error_message': 'GPS coordinate conversion failed',
                    'ned_position': [ned_x, ned_y, ned_z],
                    'ned_rotation': [ned_qw, ned_qx, ned_qy, ned_qz],
                    'motive_position': [rel_x, rel_y, rel_z],
                    'motive_rotation': [motive_qx, motive_qy, motive_qz, motive_qw],
                    'data_no': self.data_No,
                    'timestamp': data_time,
                    'frame_time': now_time
                }
                
                print(f"ERROR: GPS conversion failed for ID {new_id}")

            # **UDP送信実行（毎フレーム = 50Hz）**
            if new_id in self.udp_targets:
                target_ip = self.udp_targets[new_id]
                
                # 毎フレーム送信（50Hz）
                success = self.send_udp_data(gps_data, target_ip, new_id)
                
                # 50回に1回のみ送信確認表示
                if self.data_No % 50 == 0:
                    print(f"[Frame {self.data_No}] GPS data sent to {target_ip} (50Hz)")
                
                if not success:
                    print(f"Failed to send UDP data for RB{new_id} to {target_ip}")
            else:
                print(f"WARNING: No UDP target configured for rigid body {new_id}")

            # データをバッファに格納（コンソール表示用）
            self.data_buffer[new_id] = {
                'id': new_id,
                'position': pos,
                'rotation': rot,
                'ned_position': [ned_x, ned_y, ned_z],
                'ned_rotation': [ned_qw, ned_qx, ned_qy, ned_qz],
                'gps_coords': [gps_lat, gps_lon, gps_alt] if gps_lat is not None else None,
                'yaw_degrees': yaw_deg,
                'data_no': self.data_No,
                'time': data_time
            }

            # データ番号をインクリメント
            if new_id == 2:
                self.data_No = self.data_No + 1

            # IDが1と2のデータが揃ったらコンソール表示（簡略化）
            if 1 in self.data_buffer and 2 in self.data_buffer:
                self.data_buffer.clear()

        rigid_body = MoCapData.RigidBody(new_id, pos, rot)

        # Send information to any listener.
        if self.rigid_body_listener is not None:
            self.rigid_body_listener( new_id, pos, rot )

        # RB Marker Data処理（簡略化）
        if( major < 3 and major != 0) :
            if len(data) < offset + 4:
                return offset, rigid_body

            marker_count = int.from_bytes( data[offset:offset+4], byteorder='little', signed=True )
            offset += 4
            marker_count_range = range( 0, marker_count )

            rb_marker_list=[]
            for i in marker_count_range:
                rb_marker_list.append(MoCapData.RigidBodyMarker())

            # Marker positions
            for i in marker_count_range:
                if len(data) < offset + 12:
                    break
                pos_m = Vector3.unpack( data[offset:offset+12] )
                offset += 12
                rb_marker_list[i].pos=pos_m

            if major >= 2:
                # Marker ID's
                for i in marker_count_range:
                    if len(data) < offset + 4:
                        break
                    new_id_m = int.from_bytes( data[offset:offset+4], byteorder='little', signed=True )
                    offset += 4
                    rb_marker_list[i].id=new_id_m

                # Marker sizes
                for i in marker_count_range:
                    if len(data) < offset + 4:
                        break
                    size = FloatValue.unpack( data[offset:offset+4] )
                    offset += 4
                    rb_marker_list[i].size=size

            for i in marker_count_range:
                rigid_body.add_rigid_body_marker(rb_marker_list[i])

        if major >= 2 :
            if len(data) >= offset + 4:
                marker_error, = FloatValue.unpack( data[offset:offset+4] )
                offset += 4
                rigid_body.error = marker_error

        # Version 2.6 and later
        if ( ( major == 2 ) and ( minor >= 6 ) ) or major > 2 :
            if len(data) >= offset + 2:
                param, = struct.unpack( 'h', data[offset:offset+2] )
                tracking_valid = ( param & 0x01 ) != 0
                offset += 2

                if tracking_valid:
                    rigid_body.tracking_valid = True
                else:
                    rigid_body.tracking_valid = False
            else:
                rigid_body.tracking_valid = True

        return offset, rigid_body

    # 簡略化されたその他のunpackメソッド
    def __unpack_skeleton( self, data, major, minor, skeleton_num=0):
        offset = 0
        new_id = int.from_bytes( data[offset:offset+4], byteorder='little', signed=True )
        offset += 4
        skeleton = MoCapData.Skeleton(new_id)
        rigid_body_count = int.from_bytes( data[offset:offset+4], byteorder='little', signed=True )
        offset += 4
        if(rigid_body_count > 0):
            for rb_num in range( 0, rigid_body_count ):
                offset_tmp, rigid_body = self.__unpack_rigid_body( data[offset:], major, minor, rb_num )
                if rigid_body is not None:
                    skeleton.add_rigid_body(rigid_body)
                offset+=offset_tmp
        return offset, skeleton

    def __unpack_frame_prefix_data( self, data):
        offset = 0
        frame_number = int.from_bytes( data[offset:offset+4], byteorder='little', signed=True )
        offset += 4
        frame_prefix_data=MoCapData.FramePrefixData(frame_number)
        return offset, frame_prefix_data

    def __unpack_data_size(self, data, major, minor):
        sizeInBytes=0
        offset=0
        if( ( (major == 4) and (minor>0) ) or (major > 4)):
            sizeInBytes = int.from_bytes( data[offset:offset+4], byteorder='little', signed=True )
            offset += 4
        return offset, sizeInBytes

    def __unpack_legacy_other_markers( self, data, packet_size, major, minor):
        offset = 0
        other_marker_count = int.from_bytes( data[offset:offset+4], byteorder='little', signed=True )
        offset += 4
        offset_tmp, unpackedDataSize = self.__unpack_data_size(data[offset:],major, minor)
        offset += offset_tmp
        other_marker_data = MoCapData.LegacyMarkerData()
        if(other_marker_count > 0):
            for j in range( 0, other_marker_count ):
                pos = Vector3.unpack( data[offset:offset+12] )
                offset += 12
                other_marker_data.add_pos(pos)
        return offset, other_marker_data

    def __unpack_marker_set_data( self, data, packet_size, major, minor):
        marker_set_data=MoCapData.MarkerSetData()
        offset = 0
        marker_set_count = int.from_bytes( data[offset:offset+4], byteorder='little', signed=True )
        offset += 4
        offset_tmp, unpackedDataSize = self.__unpack_data_size(data[offset:],major, minor)
        offset += offset_tmp
        for i in range( 0, marker_set_count ):
            marker_data = MoCapData.MarkerData()
            model_name, separator, remainder = bytes(data[offset:]).partition( b'\0' )
            offset += len( model_name ) + 1
            marker_data.set_model_name(model_name)
            marker_count = int.from_bytes( data[offset:offset+4], byteorder='little', signed=True )
            offset += 4
            if(marker_count < 0):
                offset = len(data)
                return offset, marker_set_data
            elif(marker_count > 10000):
                offset = len(data)
                return offset, marker_set_data
            for j in range( 0, marker_count ):
                if(len(data)<(offset+12)):
                    offset = len(data)
                    return offset, marker_set_data
                    break
                pos = Vector3.unpack( data[offset:offset+12] )
                offset += 12
                marker_data.add_pos(pos)
            marker_set_data.add_marker_data(marker_data)
        return offset, marker_set_data

    def __unpack_rigid_body_data( self, data, packet_size, major, minor):
        rigid_body_data = MoCapData.RigidBodyData()
        offset = 0
        rigid_body_count = int.from_bytes( data[offset:offset+4], byteorder='little', signed=True )
        offset += 4
        offset_tmp, unpackedDataSize = self.__unpack_data_size(data[offset:],major, minor)
        offset += offset_tmp
        for i in range( 0, rigid_body_count ):
            offset_tmp, rigid_body = self.__unpack_rigid_body( data[offset:], major, minor, i )
            offset += offset_tmp
            if rigid_body is not None:
                rigid_body_data.add_rigid_body(rigid_body)
        return offset, rigid_body_data

    def __unpack_skeleton_data( self, data, packet_size, major, minor):
        skeleton_data = MoCapData.SkeletonData()
        offset = 0
        skeleton_count = 0
        if( ( major == 2 and minor > 0 ) or major > 2 ):
            skeleton_count = int.from_bytes( data[offset:offset+4], byteorder='little', signed=True )
            offset += 4
            offset_tmp, unpackedDataSize = self.__unpack_data_size(data[offset:],major, minor)
            offset += offset_tmp
            if(skeleton_count >0):
                for skeleton_num in range( 0, skeleton_count ):
                    rel_offset, skeleton = self.__unpack_skeleton( data[offset:], major, minor, skeleton_num )
                    offset += rel_offset
                    skeleton_data.add_skeleton(skeleton)
        return offset, skeleton_data

    def __decode_marker_id(self, new_id):
        model_id = new_id >> 16
        marker_id = new_id & 0x0000ffff
        return model_id, marker_id

    def __unpack_labeled_marker_data( self, data, packet_size, major, minor):
        labeled_marker_data = MoCapData.LabeledMarkerData()
        offset = 0
        labeled_marker_count = 0
        if( ( major == 2 and minor > 3 ) or major > 2 ):
            labeled_marker_count = int.from_bytes( data[offset:offset+4], byteorder='little', signed=True )
            offset += 4
            offset_tmp, unpackedDataSize = self.__unpack_data_size(data[offset:],major, minor)
            offset += offset_tmp
            for lm_num in range( 0, labeled_marker_count ):
                tmp_id = int.from_bytes( data[offset:offset+4], byteorder='little', signed=True )
                offset += 4
                model_id, marker_id = self.__decode_marker_id(tmp_id)
                pos = Vector3.unpack( data[offset:offset+12] )
                offset += 12
                size = FloatValue.unpack( data[offset:offset+4] )
                offset += 4
                param = 0
                if( ( major == 2 and minor >= 6 ) or major > 2):
                    param, = struct.unpack( 'h', data[offset:offset+2] )
                    offset += 2
                residual = 0.0
                if major >= 3 :
                    residual, = FloatValue.unpack( data[offset:offset+4] )
                    offset += 4
                    residual = residual * 1000.0
                labeled_marker = MoCapData.LabeledMarker(tmp_id,pos,size,param, residual)
                labeled_marker_data.add_labeled_marker(labeled_marker)
        return offset, labeled_marker_data

    def __unpack_force_plate_data( self, data, packet_size, major, minor):
        force_plate_data = MoCapData.ForcePlateData()
        offset = 0
        force_plate_count = 0
        if( ( major == 2 and minor >= 9 ) or major > 2 ):
            if len(data) < 4:
                return offset, force_plate_data
            force_plate_count = int.from_bytes( data[offset:offset+4], byteorder='little', signed=True )
            offset += 4
            if force_plate_count <= 0:
                return offset, force_plate_data
            offset_tmp, unpackedDataSize = self.__unpack_data_size(data[offset:],major, minor)
            offset += offset_tmp
            for i in range( 0, force_plate_count ):
                if len(data) < offset + 8:
                    break
                force_plate_id = int.from_bytes( data[offset:offset+4], byteorder='little', signed=True )
                offset += 4
                force_plate = MoCapData.ForcePlate(force_plate_id)
                force_plate_channel_count = int.from_bytes( data[offset:offset+4], byteorder='little', signed=True )
                offset += 4
                for j in range( force_plate_channel_count ):
                    fp_channel_data = MoCapData.ForcePlateChannelData()
                    if len(data) < offset + 4:
                        break
                    force_plate_channel_frame_count = int.from_bytes( data[offset:offset+4], byteorder='little', signed=True )
                    offset += 4
                    for k in range( force_plate_channel_frame_count ):
                        if len(data) < offset + 4:
                            break
                        force_plate_channel_val = FloatValue.unpack( data[offset:offset+4] )
                        offset += 4
                        fp_channel_data.add_frame_entry(force_plate_channel_val)
                    force_plate.add_channel_data(fp_channel_data)
                force_plate_data.add_force_plate(force_plate)
        return offset, force_plate_data

    def __unpack_device_data( self, data, packet_size, major, minor):
        device_data = MoCapData.DeviceData()
        offset = 0
        device_count = 0
        if ( major == 2 and minor >= 11 ) or (major > 2) :
            device_count = int.from_bytes( data[offset:offset+4], byteorder='little', signed=True )
            offset += 4
            offset_tmp, unpackedDataSize = self.__unpack_data_size(data[offset:],major, minor)
            offset += offset_tmp
            for i in range( 0, device_count ):
                device_id = int.from_bytes( data[offset:offset+4], byteorder='little', signed=True )
                offset += 4
                device = MoCapData.Device(device_id)
                device_channel_count = int.from_bytes( data[offset:offset+4], byteorder='little', signed=True )
                offset += 4
                for j in range( 0, device_channel_count ):
                    device_channel_data = MoCapData.DeviceChannelData()
                    device_channel_frame_count = int.from_bytes( data[offset:offset+4], byteorder='little', signed=True )
                    offset += 4
                    for k in range( 0, device_channel_frame_count ):
                        device_channel_val = FloatValue.unpack( data[offset:offset+4] )
                        offset += 4
                        device_channel_data.add_frame_entry(device_channel_val)
                    device.add_channel_data(device_channel_data)
                device_data.add_device(device)
        return offset, device_data

    def __unpack_frame_suffix_data( self, data, packet_size, major, minor):
        frame_suffix_data = MoCapData.FrameSuffixData()
        offset = 0
        timecode = int.from_bytes( data[offset:offset+4], byteorder='little', signed=True )
        offset += 4
        frame_suffix_data.timecode = timecode
        timecode_sub = int.from_bytes( data[offset:offset+4], byteorder='little', signed=True )
        offset += 4
        frame_suffix_data.timecode_sub = timecode_sub
        if((packet_size-offset) <= 0):
            pass
        else:
            if ( major == 2 and minor >= 7 ) or (major > 2 ):
                timestamp, = DoubleValue.unpack( data[offset:offset+8] )
                offset += 8
            else:
                timestamp, = FloatValue.unpack( data[offset:offset+4] )
                offset += 4
            frame_suffix_data.timestamp = timestamp
            if major >= 3 :
                stamp_camera_mid_exposure = int.from_bytes( data[offset:offset+8], byteorder='little', signed=True )
                offset += 8
                frame_suffix_data.stamp_camera_mid_exposure = stamp_camera_mid_exposure
                stamp_data_received = int.from_bytes( data[offset:offset+8], byteorder='little', signed=True )
                offset += 8
                frame_suffix_data.stamp_data_received = stamp_data_received
                stamp_transmit = int.from_bytes( data[offset:offset+8], byteorder='little', signed=True )
                offset += 8
                frame_suffix_data.stamp_transmit = stamp_transmit
            if major >= 4:
                prec_timestamp_secs = int.from_bytes( data[offset:offset+4], byteorder='little', signed=True )
                offset += 4
                frame_suffix_data.prec_timestamp_secs = prec_timestamp_secs
                prec_timestamp_frac_secs = int.from_bytes( data[offset:offset+4], byteorder='little', signed=True )
                offset += 4
                frame_suffix_data.prec_timestamp_frac_secs = prec_timestamp_frac_secs
            param, = struct.unpack( 'h', data[offset:offset+2] )
            offset += 2
            is_recording = ( param & 0x01 ) != 0
            tracked_models_changed = ( param & 0x02 ) != 0
            frame_suffix_data.param = param
            frame_suffix_data.is_recording = is_recording
            frame_suffix_data.tracked_models_changed = tracked_models_changed
        return offset, frame_suffix_data

    def __unpack_mocap_data( self, data : bytes, packet_size, major, minor):
        mocap_data = MoCapData.MoCapData()
        data = memoryview( data )
        offset = 0
        rel_offset = 0

        #Frame Prefix Data
        rel_offset, frame_prefix_data = self.__unpack_frame_prefix_data(data[offset:])
        offset += rel_offset
        mocap_data.set_prefix_data(frame_prefix_data)
        frame_number = frame_prefix_data.frame_number

        #Markerset Data
        rel_offset, marker_set_data =self.__unpack_marker_set_data(data[offset:], (packet_size - offset),major, minor)
        offset += rel_offset
        mocap_data.set_marker_set_data(marker_set_data)
        marker_set_count = marker_set_data.get_marker_set_count()
        unlabeled_markers_count = marker_set_data.get_unlabeled_marker_count()

        # Legacy Other Markers
        rel_offset, legacy_other_markers =self.__unpack_legacy_other_markers(data[offset:], (packet_size - offset),major, minor)
        offset += rel_offset
        mocap_data.set_legacy_other_markers(legacy_other_markers)
        marker_set_count = legacy_other_markers.get_marker_count()
        legacy_other_markers_count = marker_set_data.get_unlabeled_marker_count()

        # Rigid Body Data
        rel_offset, rigid_body_data = self.__unpack_rigid_body_data(data[offset:], (packet_size - offset),major, minor)
        offset += rel_offset
        mocap_data.set_rigid_body_data(rigid_body_data)
        rigid_body_count = rigid_body_data.get_rigid_body_count()

        # Skeleton Data
        rel_offset, skeleton_data = self.__unpack_skeleton_data(data[offset:], (packet_size - offset),major, minor)
        offset += rel_offset
        mocap_data.set_skeleton_data(skeleton_data)
        skeleton_count = skeleton_data.get_skeleton_count()

        # Assets処理（簡略化）
        asset_count=0

        # Labeled Marker Data
        rel_offset, labeled_marker_data = self.__unpack_labeled_marker_data(data[offset:], (packet_size - offset),major, minor)
        offset += rel_offset
        mocap_data.set_labeled_marker_data(labeled_marker_data)
        labeled_marker_count = labeled_marker_data.get_labeled_marker_count()

        # Force Plate Data
        rel_offset, force_plate_data = self.__unpack_force_plate_data(data[offset:], (packet_size - offset),major, minor)
        offset += rel_offset
        mocap_data.set_force_plate_data(force_plate_data)

        # Device Data
        rel_offset,device_data = self.__unpack_device_data(data[offset:], (packet_size - offset),major, minor)
        offset += rel_offset
        mocap_data.set_device_data(device_data)

        # Frame Suffix Data
        rel_offset, frame_suffix_data = self.__unpack_frame_suffix_data(data[offset:], (packet_size - offset),major, minor)
        offset += rel_offset
        mocap_data.set_suffix_data(frame_suffix_data)
        timecode = frame_suffix_data.timecode
        timecode_sub= frame_suffix_data.timecode_sub
        timestamp = frame_suffix_data.timestamp
        is_recording = frame_suffix_data.is_recording
        tracked_models_changed = frame_suffix_data.tracked_models_changed

        # Send information to any listener.
        if self.new_frame_listener is not None:
            data_dict={}
            data_dict["frame_number"]=frame_number
            data_dict[ "marker_set_count"] = marker_set_count
            data_dict[ "unlabeled_markers_count"] = unlabeled_markers_count
            data_dict[ "rigid_body_count"] = rigid_body_count
            data_dict[ "skeleton_count"] =skeleton_count
            data_dict[ "asset_count"] =asset_count
            data_dict[ "labeled_marker_count"] = labeled_marker_count
            data_dict[ "timecode"] = timecode
            data_dict[ "timecode_sub"] = timecode_sub
            data_dict[ "timestamp"] = timestamp
            data_dict[ "is_recording"] = is_recording
            data_dict[ "tracked_models_changed"] = tracked_models_changed

            self.new_frame_listener( data_dict )

        return offset, mocap_data

    def __process_message( self, data : bytes, print_level=0):
        major = self.get_major()
        minor = self.get_minor()

        message_id = get_message_id(data)
        packet_size = int.from_bytes( data[2:4], byteorder='little', signed=True )
        offset = 4

        if message_id == self.NAT_FRAMEOFDATA :
            offset_tmp, mocap_data = self.__unpack_mocap_data( data[offset:], packet_size, major, minor )
            offset += offset_tmp

        elif message_id == self.NAT_MODELDEF :
            pass

        elif message_id == self.NAT_SERVERINFO :
            offset += self.__unpack_server_info( data[offset:], packet_size, major, minor)

        elif message_id == self.NAT_RESPONSE :
            if packet_size == 4 :
                command_response = int.from_bytes( data[offset:offset+4], byteorder='little', signed=True )
                offset += 4
            else:
                message, separator, remainder = bytes(data[offset:]).partition( b'\0' )
                if(len(message) < 30):
                    tmpString = message.decode('utf-8')
                    if( tmpString.startswith('Bitstream') ):
                        nn_version = self.__unpack_bitstream_info(data[offset:],packet_size, major, minor)
                        if(len(nn_version)>1):
                            for i in range( len(nn_version) ):
                                self.__nat_net_stream_version_server[i] = int(nn_version[i])
                            for i in range( len(nn_version),4 ):
                                self.__nat_net_stream_version_server[i] = 0
                offset += len( message ) + 1

        elif message_id == self.NAT_UNRECOGNIZED_REQUEST :
            pass

        elif message_id == self.NAT_MESSAGESTRING :
            message, separator, remainder = bytes(data[offset:]).partition( b'\0' )
            offset += len( message ) + 1

        return message_id

    def __unpack_server_info(self, data, packet_size, major, minor):
        offset = 0

        # Server name
        self.__application_name, separator, remainder = bytes(data[offset: offset+256]).partition( b'\0' )
        self.__application_name=str(self.__application_name, "utf-8")
        offset += 256

        # Server Version info
        server_version = struct.unpack( 'BBBB', data[offset:offset+4] )
        offset += 4

        self.__server_version[0] = server_version[0]
        self.__server_version[1] = server_version[1]
        self.__server_version[2] = server_version[2]
        self.__server_version[3] = server_version[3]

        # NatNet Version info
        nnsvs = struct.unpack( 'BBBB', data[offset:offset+4] )
        offset += 4

        self.__nat_net_stream_version_server[0]=nnsvs[0]
        self.__nat_net_stream_version_server[1]=nnsvs[1]
        self.__nat_net_stream_version_server[2]=nnsvs[2]
        self.__nat_net_stream_version_server[3]=nnsvs[3]

        if (self.__nat_net_requested_version[0] == 0) and\
           (self.__nat_net_requested_version[1] == 0):
            print("resetting requested version to %d %d %d %d from %d %d %d %d"%(
                self.__nat_net_stream_version_server[0],
                self.__nat_net_stream_version_server[1],
                self.__nat_net_stream_version_server[2],
                self.__nat_net_stream_version_server[3],
                self.__nat_net_requested_version[0],
                self.__nat_net_requested_version[1],
                self.__nat_net_requested_version[2],
                self.__nat_net_requested_version[3]))

            self.__nat_net_requested_version[0] = self.__nat_net_stream_version_server[0]
            self.__nat_net_requested_version[1] = self.__nat_net_stream_version_server[1]
            self.__nat_net_requested_version[2] = self.__nat_net_stream_version_server[2]
            self.__nat_net_requested_version[3] = self.__nat_net_stream_version_server[3]

        # Determine if the bitstream version can be changed
        if (self.__nat_net_stream_version_server[0] >= 4) and (self.use_multicast == False):
            self.__can_change_bitstream_version = True

        return offset

    def __unpack_bitstream_info(self, data, packet_size, major, minor):
        nn_version=[]
        inString = data.decode('utf-8')
        messageList = inString.split(',')

        if( len(messageList) > 1 ):
            if( messageList[0] == 'Bitstream'):
                nn_version=messageList[1].split('.')

        return nn_version

    def __command_thread_function( self, in_socket, stop, gprint_level):
        message_id_dict={}

        if not self.use_multicast:
            in_socket.settimeout(2.0)

        data=bytearray(0)
        recv_buffer_size=64*1024

        while not stop():
            try:
                data, addr = in_socket.recvfrom( recv_buffer_size )
            except socket.error as msg:
                if stop():
                    break
                else:
                    print(f"ERROR: command socket access error occurred: {msg}")
                    return 1
            except socket.herror:
                print("ERROR: command socket access herror occurred")
                return 2
            except socket.gaierror:
                print("ERROR: command socket access gaierror occurred")
                return 3
            except socket.timeout:
                if(self.use_multicast):
                    print("ERROR: command socket access timeout occurred. Server not responding")
                    return 4

            if len( data ) > 0 :
                message_id = get_message_id(data)
                tmp_str="mi_%1.1d"%message_id
                if tmp_str not in message_id_dict:
                    message_id_dict[tmp_str]=0
                message_id_dict[tmp_str] += 1
                print_level = gprint_level()
                if message_id == self.NAT_FRAMEOFDATA:
                    if print_level > 0:
                        if (message_id_dict[tmp_str] % print_level) == 0:
                            print_level = 1
                        else:
                            print_level = 0
                message_id = self.__process_message( data , print_level)
                data=bytearray(0)

            if not self.use_multicast:
                if not stop():
                    self.send_keep_alive(in_socket, self.server_ip_address, self.command_port)

        return 0

    def __data_thread_function( self, in_socket, stop, gprint_level):
        message_id_dict={}
        data=bytearray(0)
        recv_buffer_size=64*1024

        while not stop():
            try:
                data, addr = in_socket.recvfrom( recv_buffer_size )
            except socket.error as msg:
                if not stop():
                    print(f"ERROR: data socket access error occurred: {msg}")
                    return 1
            except socket.herror:
                print("ERROR: data socket access herror occurred")
                return 2
            except socket.gaierror:
                print("ERROR: data socket access gaierror occurred")
                return 3
            except socket.timeout:
                print("ERROR: data socket access timeout occurred. Server not responding")
                return 4

            if len( data ) > 0 :
                message_id = get_message_id(data)
                tmp_str="mi_%1.1d"%message_id
                if tmp_str not in message_id_dict:
                    message_id_dict[tmp_str]=0
                message_id_dict[tmp_str] += 1
                print_level = gprint_level()
                if message_id == self.NAT_FRAMEOFDATA:
                    if print_level > 0:
                        if (message_id_dict[tmp_str] % print_level) == 0:
                            print_level = 1
                        else:
                            print_level = 0
                message_id = self.__process_message( data , print_level)
                data=bytearray(0)

        return 0

    def send_request( self, in_socket, command, command_str, address ):
        packet_size = 0
        if command == self.NAT_REQUEST_MODELDEF or command == self.NAT_REQUEST_FRAMEOFDATA :
            packet_size = 0
            command_str = ""
        elif command == self.NAT_REQUEST :
            packet_size = len( command_str ) + 1
        elif command == self.NAT_CONNECT :
            tmp_version=[4,1,0,0]
            print("NAT_CONNECT to Motive with %d %d %d %d\n"%(
                tmp_version[0],
                tmp_version[1],
                tmp_version[2],
                tmp_version[3]
            ))
            command_str = [0 for i in range(270)]
            command_str[0] =80
            command_str[1] =105
            command_str[2] =110
            command_str[3] =103
            command_str[264] =0
            command_str[265] =tmp_version[0]
            command_str[266] =tmp_version[1]
            command_str[267] =tmp_version[2]
            command_str[268] =tmp_version[3]
            packet_size = len( command_str ) + 1
        elif command == self.NAT_KEEPALIVE:
            packet_size = 0
            command_str = ""

        data = command.to_bytes( 2, byteorder='little', signed=True )
        data += packet_size.to_bytes( 2, byteorder='little', signed=True )

        if command == self.NAT_CONNECT :
            data+=bytearray(command_str)
        else:
            data += command_str.encode( 'utf-8' )
            data += b'\0'

        return in_socket.sendto( data, address )

    def send_command( self, command_str):
        nTries = 3
        ret_val = -1

        while nTries:
            nTries -= 1
            ret_val = self.send_request( self.command_socket, self.NAT_REQUEST, command_str, (self.server_ip_address, self.command_port) )

            if (ret_val != -1):
                break;

        return ret_val

    def send_commands(self,tmpCommands, print_results: bool =True):
        for sz_command in tmpCommands:
            return_code = self.send_command(sz_command)

            if(print_results):
                print("Command: %s - return_code: %d"% (sz_command, return_code) )

    def send_keep_alive(self,in_socket, server_ip_address, server_port):
        return self.send_request(in_socket, self.NAT_KEEPALIVE, "", (server_ip_address, server_port))

    def get_command_port(self):
        return self.command_port

    def refresh_configuration(self):
        sz_command = "Bitstream"
        return_code = self.send_command(sz_command)
        time.sleep(0.5)

    def get_application_name(self):
        return self.__application_name

    def get_nat_net_requested_version(self):
        return self.__nat_net_requested_version

    def get_nat_net_version_server(self):
        return self.__nat_net_stream_version_server

    def get_server_version(self):
        return self.__server_version

    def run( self ):
        print("Starting Dual Rigid Body GPS Transmission System...")
        print(f"GPS reference: ({self.ref_lat:.7f}, {self.ref_lon:.7f}, {self.ref_alt:.3f})")
        print("UDP transmission targets:")
        for rb_id, ip in self.udp_targets.items():
            print(f"  Rigid Body {rb_id} → {ip}:{self.udp_port}")
        print("UDP sending at 50Hz (every frame), Console display every 50 frames")
        print("Waiting for rigid body data from Motive...")
        print("Press Ctrl+C to stop\n")

        # Create the data socket
        self.data_socket = self.__create_data_socket( self.data_port )
        if self.data_socket is None :
            print("Could not open data channel")
            return False

        # Create the command socket
        self.command_socket = self.__create_command_socket()
        if self.command_socket is None :
            print("Could not open command channel")
            return False

        self.__is_locked = True
        self.stop_threads = False

        # Create a separate thread for receiving data packets
        self.data_thread = Thread( target = self.__data_thread_function, args = (self.data_socket, lambda : self.stop_threads, lambda : self.print_level, ))
        self.data_thread.start()

        # Create a separate thread for receiving command packets
        self.command_thread = Thread( target = self.__command_thread_function, args = (self.command_socket, lambda : self.stop_threads, lambda : self.print_level,))
        self.command_thread.start()

        # Create a keyboard listener thread for recording
        if self.recording_enabled:
            self.keyboard_thread = Thread(target=self.keyboard_listener, daemon=True)
            self.keyboard_thread.start()

        # Required for setup
        # Get NatNet and server versions
        print("Attempting to connect to Motive...")
        self.send_request(self.command_socket, self.NAT_CONNECT, "", (self.server_ip_address, self.command_port) )

        return True

    def shutdown(self):
        print("Shutdown called")
        print(f"UDP Statistics - Sent: {self.udp_send_count}, Errors: {self.udp_error_count}")
        self.stop_threads = True

        try:
            self.command_socket.close()
            self.data_socket.close()
        except:
            pass

        try:
            self.command_thread.join()
            self.data_thread.join()
        except:
            pass

        sys.exit()
