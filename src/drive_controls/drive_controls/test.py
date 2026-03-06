# id_scan.py
import serial, time

PORT = "/dev/ttyUSB0"
BAUD = 115200

def make_packet(device_id, func_request, param=0):
    complement = (257 - device_id) & 0xFF
    frame = bytes([0xFF, device_id, 0x03, func_request, param])
    cs = (sum(frame) + complement) & 0xFF
    return frame + bytes([cs])

# We'll use FUNC_REQUEST_DATA (0x50) with argument FUNC_VERSION (0x51)
FUNC_REQUEST = 0x50
FUNC_VERSION = 0x51

ser = serial.Serial(PORT, BAUD, timeout=0.1)
ser.reset_input_buffer()
print("Scanning device IDs 1..255 (this may take ~20s)...")
for did in range(1, 256):
    pkt = make_packet(did, FUNC_REQUEST, FUNC_VERSION)
    ser.write(pkt)
    ser.flush()
    time.sleep(0.02)  # small gap
    resp = ser.read(128)
    if resp:
        print(f"ID 0x{did:02X} -> response ({len(resp)} bytes):", ' '.join(f'{b:02X}' for b in resp))
ser.close()
print("Scan done.")