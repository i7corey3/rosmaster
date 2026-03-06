# measure_enc_cpr.py
# Requires: pip install pyserial
import serial
import time
import struct

HEAD = 0xFF
DEVICE_ID = 0xFC  # default device id used by Yahboom Rosmaster
FUNC_REQUEST_ENCODER = 0x0D

def complement(device_id):
    return (257 - device_id) & 0xFF

def compute_checksum(cmd_bytes, device_id=DEVICE_ID):
    s = complement(device_id)
    for b in cmd_bytes:
        s = (s + b) & 0xFF
    return s

def build_request_data(function, param=0, device_id=DEVICE_ID):
    base = bytearray([HEAD, device_id, 0x05, 0x50, function, param])
    chk = compute_checksum(base, device_id)
    base.append(chk)
    # In some Python drivers the second byte is device_id-1; if you need that, adjust.
    return bytes(base)

def parse_encoder_frame(frame, expected_device_id=DEVICE_ID):
    # minimal check
    if len(frame) < 6: return None
    if frame[0] != HEAD: return None
    # some firmware sends device_id-1 in second byte; accept both
    dev_byte = frame[1]
    if dev_byte not in (expected_device_id, (expected_device_id - 1) & 0xFF):
        return None
    ext_len = frame[2]
    ext_type = frame[3]
    if ext_type != FUNC_REQUEST_ENCODER: return None
    data_len = ext_len - 2
    if data_len < 1: return None
    payload_len = data_len - 1
    if payload_len < 16: return None
    # compute checksum
    chk = frame[4+payload_len]
    s = (ext_len + ext_type + sum(frame[4:4+payload_len])) & 0xFF
    if s != chk:
        return None
    # decode 4 little-endian int32
    payload = frame[4:4+payload_len]
    enc = struct.unpack('<4i', payload[0:16])
    return enc

def read_response(ser, timeout=1.0):
    deadline = time.time() + timeout
    buf = bytearray()
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        buf += b
        # try parse anywhere in buffer
        for i in range(max(0, len(buf)-64)):
            if buf[i] != HEAD:
                continue
            # ensure we have at least 4 bytes to read ext_len
            if i+3 >= len(buf): break
            ext_len = buf[i+2]
            if ext_len < 2: 
                continue
            data_len = ext_len - 2
            total_len = 4 + data_len
            if i + total_len > len(buf): 
                break
            frame = bytes(buf[i:i+total_len])
            parsed = parse_encoder_frame(frame)
            if parsed is not None:
                return parsed
    return None

def main():
    port = '/dev/ttyUSB0'
    baud = 115200
    ser = serial.Serial(port, baudrate=baud, timeout=0.05)
    time.sleep(0.1)
    print("Requesting encoder snapshot 1...")
    ser.write(build_request_data(FUNC_REQUEST_ENCODER))
    enc1 = read_response(ser, timeout=1.0)
    print("Enc1:", enc1)
    input("Rotate wheel exactly one revolution (press Enter when done) ...")
    time.sleep(0.1)
    ser.write(build_request_data(FUNC_REQUEST_ENCODER))
    enc2 = read_response(ser, timeout=1.0)
    print("Enc2:", enc2)
    if enc1 and enc2:
        diffs = [c2 - c1 for c1,c2 in zip(enc1,enc2)]
        print("Diffs:", diffs)
        print("Suggested counts-per-rev (for each encoder):", [abs(d) for d in diffs])
    else:
        print("Failed to read encoder frames. Check connection or timeouts.")

if __name__ == '__main__':
    main()