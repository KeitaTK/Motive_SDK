# とりあえず表示を減らしたやつ

import sys
import time
from NatNetClient import NatNetClient
import DataDescriptions
import MoCapData
import socket
import threading
import pickle
import msvcrt  # Windows用の非ブロッキング入力

#   グローバル変数
new_id  = 0
pos  = 0
rot = 0
is_looping = True

# This is a callback function that gets connected to the NatNet client
# and called once per mocap frame.
def receive_new_frame(data_dict):
    order_list=[ "frameNumber", "markerSetCount", "unlabeledMarkersCount", "rigidBodyCount", "skeletonCount",
                "labeledMarkerCount", "timecode", "timecodeSub", "timestamp", "isRecording", "trackedModelsChanged" ]
    dump_args = False
    if dump_args == True:
        out_string = "    "
        for key in data_dict:
            out_string += key + "="
            if key in data_dict :
                out_string += data_dict[key] + " "
            out_string+="/"
        print(out_string)

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame( new_id, position, rotation ):
    pass
    #print( "Received frame for rigid body", new_id )
    #print( "Received frame for rigid body", new_id," ",position," ",rotation )

def add_lists(totals, totals_tmp):
    totals[0]+=totals_tmp[0]
    totals[1]+=totals_tmp[1]
    totals[2]+=totals_tmp[2]
    return totals

def print_configuration(natnet_client):
    natnet_client.refresh_configuration()
    print("Connection Configuration:")
    print("  Client:          %s"% natnet_client.local_ip_address)
    print("  Server:          %s"% natnet_client.server_ip_address)
    print("  Command Port:    %d"% natnet_client.command_port)
    print("  Data Port:       %d"% natnet_client.data_port)

    changeBitstreamString = "  Can Change Bitstream Version = "
    if natnet_client.use_multicast:
        print("  Using Multicast")
        print("  Multicast Group: %s"% natnet_client.multicast_address)
        changeBitstreamString+="false"
    else:
        print("  Using Unicast")
        changeBitstreamString+="true"

    #NatNet Server Info
    application_name = natnet_client.get_application_name()
    nat_net_requested_version = natnet_client.get_nat_net_requested_version()
    nat_net_version_server = natnet_client.get_nat_net_version_server()
    server_version = natnet_client.get_server_version()

    print("  NatNet Server Info")
    print("    Application Name %s" %(application_name))
    print("    MotiveVersion  %d %d %d %d"% (server_version[0], server_version[1], server_version[2], server_version[3]))
    print("    NatNetVersion  %d %d %d %d"% (nat_net_version_server[0], nat_net_version_server[1], nat_net_version_server[2], nat_net_version_server[3]))
    print("  NatNet Bitstream Requested")
    print("    NatNetVersion  %d %d %d %d"% (nat_net_requested_version[0], nat_net_requested_version[1],\
       nat_net_requested_version[2], nat_net_requested_version[3]))

    print(changeBitstreamString)
    #print("command_socket = %s"%(str(natnet_client.command_socket)))
    #print("data_socket    = %s"%(str(natnet_client.data_socket)))
    print("  PythonVersion    %s"%(sys.version))
    

def request_data_descriptions(s_client):
    # Request the model definitions
    s_client.send_request(s_client.command_socket, s_client.NAT_REQUEST_MODELDEF,    "",  (s_client.server_ip_address, s_client.command_port) )

def test_classes():
    totals = [0,0,0]
    print("Test Data Description Classes")
    totals_tmp = DataDescriptions.test_all()
    totals=add_lists(totals, totals_tmp)
    print("")
    print("Test MoCap Frame Classes")
    totals_tmp = MoCapData.test_all()
    totals=add_lists(totals, totals_tmp)
    print("")
    print("All Tests totals")
    print("--------------------")
    print("[PASS] Count = %3.1d"%totals[0])
    print("[FAIL] Count = %3.1d"%totals[1])
    print("[SKIP] Count = %3.1d"%totals[2])

def my_parse_args(arg_list, args_dict):
    # set up base values
    arg_list_len=len(arg_list)
    if arg_list_len>1:
        args_dict["serverAddress"] = arg_list[1]
        if arg_list_len>2:
            args_dict["clientAddress"] = arg_list[2]
        if arg_list_len>3:
            if len(arg_list[3]):
                args_dict["use_multicast"] = True
                if arg_list[3][0].upper() == "U":
                    args_dict["use_multicast"] = False

    return args_dict

# 非ブロッキングでキー入力を監視し、エンターで記録開始/終了をトグルするメソッド
def keyboard_monitor():
    """非ブロッキングでキーボード入力を監視"""
    global is_looping
    print("キーボード監視開始")
    print("  Enter: 記録開始/終了（トグル）")
    print("  q: プログラム終了")
    print("  h: ヘルプ表示\n")
    
    while is_looping:
        try:
            # キーが押されているかチェック（非ブロッキング）
            if msvcrt.kbhit():
                key = msvcrt.getch()
                
                # Enterキー（\r または \n）
                if key in [b'\r', b'\n']:
                    if not streaming_client.is_recording:
                        print("\n[Enter押下] 記録開始...")
                        streaming_client.start_recording()
                    else:
                        print("\n[Enter押下] 記録終了...")
                        streaming_client.stop_recording()
                
                # qキー
                elif key in [b'q', b'Q']:
                    print("\n[q押下] プログラム終了...")
                    is_looping = False
                    streaming_client.shutdown()
                    break
                
                # hキー
                elif key in [b'h', b'H']:
                    print("\n--- ヘルプ ---")
                    print("  Enter: 記録開始/終了（トグル）")
                    print("  q: プログラム終了")
                    print("  h: このヘルプを表示")
                    print("--------------\n")
            
            # CPU負荷を減らすため少し待機
            time.sleep(0.1)
            
        except KeyboardInterrupt:
            print("\nキーボード割り込み: プログラム終了...")
            is_looping = False
            streaming_client.shutdown()
            break
        except Exception as e:
            print(f"キーボード監視エラー: {e}")
            time.sleep(0.1)


#受け取ったデータを表示するメソッド　勝手にスレッドで回ってる
# def rvlocal():
#     global new_id, pos, rot
#     sv = socket.socket( socket.AF_INET )
#     sv.bind( ( "localhost", 15769 ) )
#     sv.listen()

#     while True:
#         client, addr = sv.accept()
#         data = client.recv( 1024 )
#         if len(data) == 0:
#             break
#         deserialized_deta = pickle.loads(data)

#         new_id, pos, rot = deserialized_deta

#         # print(new_id)
#         # print(pos)
#         # print(rot)

#         client.close()


if __name__ == "__main__":

    optionsDict = {}
    optionsDict["clientAddress"] = "127.0.0.1"
    optionsDict["serverAddress"] = "127.0.0.1"
    optionsDict["use_multicast"] = True

    # This will create a new NatNet client
    optionsDict = my_parse_args(sys.argv, optionsDict)

    streaming_client = NatNetClient()
    streaming_client.set_client_address(optionsDict["clientAddress"])
    streaming_client.set_server_address(optionsDict["serverAddress"])
    streaming_client.set_use_multicast(optionsDict["use_multicast"])

    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.new_frame_listener = receive_new_frame
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()
    if not is_running:
        print("ERROR: Could not start streaming client.")
        try:
            sys.exit(1)
        except SystemExit:
            print("...")
        finally:
            print("exiting")

    is_looping = True
    time.sleep(1)
    if streaming_client.connected() is False:
        print("ERROR: Could not connect properly.  Check that Motive streaming is on.")
        try:
            sys.exit(2)
        except SystemExit:
            print("...")
        finally:
            print("exiting")

    print_configuration(streaming_client)
    print("\n")
    print("=" * 50)
    print("キーボード入力待機中...")
    print("  Enter: 記録開始/終了（トグル）")
    print("  q: プログラム終了")
    print("  h: ヘルプ表示")
    print("=" * 50)
    print()

    # 非ブロッキングキーボード監視スレッドを開始
    keyboard_thread = threading.Thread(target=keyboard_monitor, daemon=True)
    keyboard_thread.start()

    # メインスレッドをアクティブに保つ
    try:
        while is_looping:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n\nプログラム終了...")
        is_looping = False
        streaming_client.shutdown()



