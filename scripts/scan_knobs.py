#!/usr/bin/env python3
"""ET08A 16通道全量检测"""
import serial, time, os, sys

PORT = "/dev/ttyUSB0"
BAUD = 100000
TRAILER = bytes([0x00, 0x04, 0x20, 0x00, 0x01, 0x08, 0x40, 0x00, 0x02, 0x10, 0x00, 0x03])

def decode(frame):
    payload = frame[2:24]
    ch = []
    for i in range(16):
        s = i * 11; b = s // 8; off = s % 8
        if b + 2 < 22:
            v = payload[b] | (payload[b+1]<<8) | (payload[b+2]<<16)
            ch.append((v >> off) & 0x7FF)
    return ch

def main():
    if not os.path.exists(PORT):
        print(f"{PORT} 不存在!"); sys.exit(1)
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(0.3); ser.read(4096)

    print("请旋转 LD 和 RD 旋钮, Ctrl+C 退出\n")
    hdr = "".join(f"CH{i+1:>6}" for i in range(16))
    print(f"{'F#':>4} {hdr}")
    print("-"*105)

    buf = b""; fc = 0; last = [0]*16
    try:
        while True:
            raw = ser.read(256)
            if not raw: time.sleep(0.002); continue
            buf += raw
            while len(buf) >= 25:
                p = buf.find(TRAILER)
                if p < 0: buf = buf[-24:]; break
                s = p - 13
                if s < 0: buf = buf[p+12:]; continue
                f = buf[s:s+25]; buf = buf[s+25:]
                ch = decode(f); fc += 1
                changed = any(abs(ch[i]-last[i]) > 10 for i in range(16))
                if changed or fc <= 3 or fc % 50 == 0:
                    vals = "".join(f"\033[1;33m{ch[i]:>6}\033[0m" if abs(ch[i]-last[i])>10 else f"{ch[i]:>6}" for i in range(16))
                    print(f"{fc:>4} {vals}")
                    if changed:
                        for i in range(16):
                            if abs(ch[i]-last[i]) > 10:
                                print(f"     CH{i+1}: {last[i]} -> {ch[i]}")
                last = ch[:]
    except KeyboardInterrupt:
        print(f"\n共 {fc} 帧")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
