#!/usr/bin/env python3
"""
diagnose_packet.py - Diagnostic tool to identify the exact NatNet data format
received from Motive.

Investigates:
  1. Whether data port sends raw frames without NatNet message header
  2. Whether NatNet 4.x data_size_prefix version mismatch occurs
  3. Whether Motive streaming excludes timestamp (motive_timestamp=-1)

Does NOT send UDP or record CSV. Auto-exits after MAX_FRAMES frames.
"""

import struct
import sys
import time

from NatNetClient import NatNetClient, get_message_id
import MoCapData

MAX_FRAMES = 10


class DiagnosticNatNetClient(NatNetClient):
    """NatNetClient subclass that adds diagnostic logging to packet processing.

    Overrides __process_message to intercept and log raw frame data, then
    re-implements the mocap parsing chain with detailed suffix diagnostics.
    """

    def __init__(self):
        super().__init__()
        self.diag_frame_count = 0

    # ------------------------------------------------------------------
    #  Override: __process_message  (name-mangled to
    #  _DiagnosticNatNetClient__process_message, so the parent's
    #  _NatNetClient__process_message is not affected)
    # ------------------------------------------------------------------
    def __process_message(self, data: bytes, print_level=0):
        major = self.get_major()
        minor = self.get_minor()
        message_id = get_message_id(data)
        packet_size = int.from_bytes(data[2:4], byteorder="little", signed=True)

        if message_id != self.NAT_FRAMEOFDATA:
            # Pass non-frame messages to the parent handler unchanged
            return self._NatNetClient__process_message(data, print_level)

        # --- Frame-of-data diagnostic ---------------------------------------
        self.diag_frame_count += 1
        fn = self.diag_frame_count

        print(f"\n{'='*70}")
        print(f"[DIAGNOSTIC]  Frame #{fn} / {MAX_FRAMES}")
        print(f"{'='*70}")

        # 1) Raw hex - first 16 bytes of the *whole* UDP payload
        raw16 = data[:16]
        print(f"  Raw packet hex (first 16 bytes): {raw16.hex(' ')}")
        print(f"  Byte[0:2]  message_id     (int16 LE): {message_id}")
        print(f"  Byte[2:4]  packet_size    (int16 LE): {packet_size}")
        print(f"  len(data)  total                     : {len(data)}")
        expected = 4 + packet_size
        print(f"  Expected   4 (header) + packet_size  : {expected}")
        if len(data) != expected:
            print(f"  *** WARNING: len(data) MISMATCH "
                  f"(got {len(data)}, expected {expected}) ***")

        # 2) Streaming version
        print(f"  Major/Minor version                  : {major}.{minor}")

        # 3) Peek at the first 32 bytes of the payload (after 4-byte header)
        payload = data[4:]
        peek_len = min(32, len(payload))
        print(f"  Payload hex (first {peek_len} bytes): "
              f"{payload[:peek_len].hex(' ')}")

        # 4) Walk through the mocap data sections, delegating to the parent's
        #    private un-packers for everything except the suffix.
        mocap_data = MoCapData.MoCapData()
        offset = 4  # past the NatNet header

        # -- Frame Prefix (4 bytes) --
        frame_number = int.from_bytes(data[offset:offset + 4],
                                      byteorder="little", signed=True)
        mocap_data.set_prefix_data(MoCapData.FramePrefixData(frame_number))
        offset += 4
        print(f"  Frame Number                         : {frame_number}")

        # -- Marker Set Data --
        rel, marker_set_data = self._NatNetClient__unpack_marker_set_data(
            data[offset:], packet_size - offset, major, minor)
        offset += rel
        mocap_data.set_marker_set_data(marker_set_data)

        # -- Legacy Other Markers --
        rel, legacy = self._NatNetClient__unpack_legacy_other_markers(
            data[offset:], packet_size - offset, major, minor)
        offset += rel
        mocap_data.set_legacy_other_markers(legacy)

        # -- Rigid Body Data --
        print(f"  Offset before rigid_body_data        : {offset}")
        rel, rigid_body_data = self._NatNetClient__unpack_rigid_body_data(
            data[offset:], packet_size - offset, major, minor)
        offset += rel
        mocap_data.set_rigid_body_data(rigid_body_data)
        print(f"  Offset after  rigid_body_data        : {offset}")

        # -- Skeleton Data --
        rel, skeleton_data = self._NatNetClient__unpack_skeleton_data(
            data[offset:], packet_size - offset, major, minor)
        offset += rel
        mocap_data.set_skeleton_data(skeleton_data)

        # -- Labeled Marker Data --
        rel, labeled_data = self._NatNetClient__unpack_labeled_marker_data(
            data[offset:], packet_size - offset, major, minor)
        offset += rel
        mocap_data.set_labeled_marker_data(labeled_data)

        # -- Force Plate Data --
        rel, fp_data = self._NatNetClient__unpack_force_plate_data(
            data[offset:], packet_size - offset, major, minor)
        offset += rel
        mocap_data.set_force_plate_data(fp_data)

        # -- Device Data --
        rel, dev_data = self._NatNetClient__unpack_device_data(
            data[offset:], packet_size - offset, major, minor)
        offset += rel
        mocap_data.set_device_data(dev_data)

        # -- Frame Suffix Data (WITH FULL DIAGNOSTICS) --
        remaining_for_suffix = packet_size - offset
        print(f"  Offset before suffix_data            : {offset}")
        print(f"  Remaining bytes for suffix           : {remaining_for_suffix}")
        rel, suffix_data = self._diag_unpack_suffix(
            data[offset:], remaining_for_suffix, major, minor)
        offset += rel
        mocap_data.set_suffix_data(suffix_data)

        # Store timestamp the same way the parent does
        self.current_frame_timestamp = suffix_data.timestamp

        # -- Rigid Body summary --
        rb_count = rigid_body_data.get_rigid_body_count()
        print(f"  Rigid Body Count                     : {rb_count}")
        for rb in rigid_body_data.rigid_body_list:
            pos = rb.pos
            rot = rb.rot
            print(f"    RB  ID={rb.id_num:>4d}  "
                  f"pos=[{pos[0]: .4f}, {pos[1]: .4f}, {pos[2]: .4f}]  "
                  f"rot=[{rot[0]: .4f}, {rot[1]: .4f}, {rot[2]: .4f}, {rot[3]: .4f}]  "
                  f"tracking_valid={rb.tracking_valid}")

        # -- Auto-exit check --
        if self.diag_frame_count >= MAX_FRAMES:
            print(f"\n{'='*70}")
            print(f"  Diagnostic complete: {MAX_FRAMES} frames analysed.")
            print(f"{'='*70}")
            self.stop_threads = True

        return message_id

    # ------------------------------------------------------------------
    #  Diagnostic suffix un-packer
    # ------------------------------------------------------------------
    @staticmethod
    def _hex(data, maxlen=64):
        """Return a space-separated hex string of the first *maxlen* bytes."""
        return data[:maxlen].hex(" ")

    def _diag_unpack_suffix(self, data: bytes, packet_size: int,
                            major: int, minor: int):
        """Unpack frame suffix data with full diagnostic output."""
        suffix = MoCapData.FrameSuffixData()
        data_len = len(data)  # actual bytes available
        offset = 0

        print(f"\n  --- Frame Suffix Diagnostics ---")
        print(f"  Suffix declared remaining (packet_size): {packet_size}")
        print(f"  Suffix actual    bytes in buffer        : {data_len}")
        print(f"  Suffix raw hex (first 64 bytes): {self._hex(data)}")

        if data_len < 8:
            print(f"  *** ERROR: Suffix buffer too small ({data_len} bytes, "
                  f"need >= 8 for timecode + timecode_sub) ***")
            return offset, suffix

        # --- timecode (int32 LE) ---
        timecode = int.from_bytes(data[offset:offset + 4],
                                  byteorder="little", signed=True)
        suffix.timecode = timecode
        offset += 4
        print(f"  [offset {offset - 4:>3d}] timecode            : {timecode}")

        # --- timecode_sub (int32 LE) ---
        timecode_sub = int.from_bytes(data[offset:offset + 4],
                                      byteorder="little", signed=True)
        suffix.timecode_sub = timecode_sub
        offset += 4
        print(f"  [offset {offset - 4:>3d}] timecode_sub        : {timecode_sub}")

        # --- Guard: is there anything after timecode_sub? ---
        if packet_size <= offset:
            print(f"  " + "-" * 49)
            print(f"  *** NO REMAINING BYTES after timecode_sub! ***")
            print(f"  *** packet_size={packet_size}, offset={offset} ***")
            print(f"  *** CONCLUSION: Motive streaming is configured")
            print(f"      WITHOUT timestamps. The timestamp field")
            print(f"      (and all subsequent suffix fields) is ABSENT. ***")
            print(f"  *** This explains motive_timestamp=-1 in CSV. ***")
            print(f"  *** FIX: In Motive, go to Edit -> Application Settings")
            print(f"      -> Streaming, and enable 'Include Timestamps'. ***")
            suffix.timestamp = -1
            return offset, suffix

        # --- Timestamp ---
        rem_after_tc = packet_size - offset
        if (major == 2 and minor >= 7) or (major > 2):
            # Double-precision (8 bytes)
            if rem_after_tc >= 8 and data_len >= offset + 8:
                timestamp = struct.unpack("<d", data[offset:offset + 8])[0]
                suffix.timestamp = timestamp
                offset += 8
                print(f"  [offset {offset - 8:>3d}] timestamp (float64 LE): {timestamp:.6f}")
            else:
                print(f"  *** ERROR: Expected double timestamp (8 bytes) but only "
                      f"{min(rem_after_tc, data_len - offset)} bytes available ***")
                suffix.timestamp = -1
                return offset, suffix
        else:
            # Float (4 bytes) - older protocol
            if rem_after_tc >= 4 and data_len >= offset + 4:
                timestamp = struct.unpack("<f", data[offset:offset + 4])[0]
                suffix.timestamp = timestamp
                offset += 4
                print(f"  [offset {offset - 4:>3d}] timestamp (float32 LE) : {timestamp:.6f}")
            else:
                print(f"  *** ERROR: Expected float timestamp (4 bytes) but only "
                      f"{min(rem_after_tc, data_len - offset)} bytes available ***")
                suffix.timestamp = -1
                return offset, suffix

        # --- v3+ fields ---
        if major >= 3:
            if data_len >= offset + 8:
                val = int.from_bytes(data[offset:offset + 8],
                                     byteorder="little", signed=True)
                suffix.stamp_camera_mid_exposure = val
                offset += 8
                print(f"  [offset {offset - 8:>3d}] stamp_camera_mid_exp : {val}")
            if data_len >= offset + 8:
                val = int.from_bytes(data[offset:offset + 8],
                                     byteorder="little", signed=True)
                suffix.stamp_data_received = val
                offset += 8
                print(f"  [offset {offset - 8:>3d}] stamp_data_received   : {val}")
            if data_len >= offset + 8:
                val = int.from_bytes(data[offset:offset + 8],
                                     byteorder="little", signed=True)
                suffix.stamp_transmit = val
                offset += 8
                print(f"  [offset {offset - 8:>3d}] stamp_transmit        : {val}")

        # --- v4+ fields ---
        if major >= 4:
            if data_len >= offset + 4:
                val = int.from_bytes(data[offset:offset + 4],
                                     byteorder="little", signed=True)
                suffix.prec_timestamp_secs = val
                offset += 4
                print(f"  [offset {offset - 4:>3d}] prec_timestamp_secs   : {val}")
            if data_len >= offset + 4:
                val = int.from_bytes(data[offset:offset + 4],
                                     byteorder="little", signed=True)
                suffix.prec_timestamp_frac_secs = val
                offset += 4
                print(f"  [offset {offset - 4:>3d}] prec_timestamp_frac_s : {val}")

        # --- param (int16) ---
        if data_len >= offset + 2:
            param = struct.unpack("<h", data[offset:offset + 2])[0]
            is_recording = (param & 0x01) != 0
            tracked_models_changed = (param & 0x02) != 0
            suffix.param = param
            suffix.is_recording = is_recording
            suffix.tracked_models_changed = tracked_models_changed
            offset += 2
            print(f"  [offset {offset - 2:>3d}] param                : {param}")
            print(f"           is_recording           : {is_recording}")
            print(f"           tracked_models_changed : {tracked_models_changed}")

        print(f"  Suffix total bytes read                : {offset}")
        if offset < packet_size:
            unread = packet_size - offset
            print(f"  *** NOTE: {unread} trailing byte(s) after suffix "
                  f"were NOT consumed ***")
            print(f"  Trailing hex: {data[offset:offset + min(unread, 32)].hex(' ')}")

        return offset, suffix


# ======================================================================
def main():
    print("=" * 70)
    print("  NatNet Packet Diagnostic Tool")
    print("=" * 70)
    print(f"  Capturing & analysing {MAX_FRAMES} frame(s)")
    print("  Target          : 127.0.0.1")
    print("  Data port       : 1511")
    print("  Command port    : 1510")
    print("  Multicast       : True  (239.255.42.99)")
    print("=" * 70)
    print()

    client = DiagnosticNatNetClient()
    client.set_client_address("127.0.0.1")
    client.set_server_address("127.0.0.1")
    client.set_use_multicast(True)

    # Prevent accidental UDP sends / CSV recording
    client.udp_targets = {}
    client.recording_enabled = False

    ok = client.run()
    if not ok:
        print("ERROR: Could not start streaming client.")
        sys.exit(1)

    time.sleep(1)
    if not client.connected():
        print("ERROR: Could not connect properly.")
        print("       Check that Motive is running and streaming is ON.")
        print("       (View -> Data Streaming, Multicast, 239.255.42.99)")
        client.shutdown()
        sys.exit(2)

    # Connection info
    sv = client.get_server_version()
    nn = client.get_nat_net_version_server()
    nr = client.get_nat_net_requested_version()
    print(f"\n  Connected: {client.get_application_name()}")
    print(f"  Motive   version : {sv[0]}.{sv[1]}.{sv[2]}.{sv[3]}")
    print(f"  NatNet   version : {nn[0]}.{nn[1]}.{nn[2]}.{nn[3]} (server)")
    print(f"  NatNet   version : {nr[0]}.{nr[1]}.{nr[2]}.{nr[3]} (requested)")
    print()

    # Wait until MAX_FRAMES are processed or timeout
    timeout_s = 30
    print(f"  Waiting for frames ... (timeout {timeout_s} s)\n")
    t0 = time.time()
    while (not client.stop_threads) and ((time.time() - t0) < timeout_s):
        time.sleep(0.1)

    if not client.stop_threads:
        print(f"\n  TIMEOUT after {timeout_s} s - no (or too few) frames "
              f"received.")
        print("  Verify that Motive is playing/recording and streaming data.")

    client.shutdown()
    print("\n  Done.")


if __name__ == "__main__":
    main()
