#!/usr/bin/env python3
"""
ET08A W.BUS 多方法解码对比脚本
"""
import serial
import time
import os
import sys

PORT = "/dev/ttyUSB0"
BAUD = 100000
TRAILER = bytes([0x00, 0x04, 0x20, 0x00, 0x01, 0x08, 0x40, 0x00, 0x02, 0x10, 0x00, 0x03])
FRAME_LEN = 25


def extract_channels_sbus(payload_22b):
    """
    标准S.BUS解码: 22字节 → 16通道 × 11-bit
    从字节流中按位偏移提取
    """
    channels = []
    for i in range(16):
        start_bit = i * 11
        byte_idx = start_bit // 8
        bit_off = start_bit % 8
        if byte_idx + 2 < len(payload_22b):
            # 读2-3个字节组装11-bit值
            val = payload_22b[byte_idx] | (payload_22b[byte_idx + 1] << 8)
            if byte_idx + 2 < len(payload_22b):
                val |= (payload_22b[byte_idx + 2] << 16)
            val = (val >> bit_off) & 0x7FF
        else:
            val = 0
        channels.append(val)
    return channels


def extract_channels_alt(payload_11b):
    """
    备选解码: 11字节 → 8通道 × 11-bit  
    使用与标准SBUS相同逻辑
    """
    channels = []
    for i in range(8):
        start_bit = i * 11
        byte_idx = start_bit // 8
        bit_off = start_bit % 8
        if byte_idx + 2 < len(payload_11b):
            val = payload_11b[byte_idx] | (payload_11b[byte_idx + 1] << 8)
            if byte_idx + 2 < len(payload_11b):
                val |= (payload_11b[byte_idx + 2] << 16)
            val = (val >> bit_off) & 0x7FF
        else:
            val = 0
        channels.append(val)
    return channels


def label(ch, val):
    is_2pos = (ch == 4 or ch == 7)  # SA, SD
    if is_2pos:
        if abs(val - 353) < 120:   return "HI "
        elif abs(val - 1694) < 120: return "LO "
        else: return f"{val:>4}"
    else:
        if abs(val - 353) < 120:   return "HI "
        elif abs(val - 1024) < 120: return "MID"
        elif abs(val - 1694) < 120: return "LO "
        else: return f"{val:>4}"


def main():
    print("=" * 90)
    print("  ET08A W.BUS 多方法解码对比")
    print("=" * 90)

    if not os.path.exists(PORT):
        print(f"[ERROR] {PORT} 不存在!")
        sys.exit(1)

    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(0.5)
    ser.read(4096)

    print("\n[INFO] 请移动摇杆和开关进行测试, 按 Ctrl+C 退出\n")
    print(f"{'Frame':>6} | {'Method':6} | {'CH1':>5} {'CH2':>5} {'CH3':>5} {'CH4':>5} | {'CH5(SA)':>6} {'CH6(SB)':>6} {'CH7(SC)':>6} {'CH8(SD)':>6}")
    print("-" * 90)

    buf = b""
    frame_count = 0
    last_method1 = None

    try:
        while True:
            raw = ser.read(1024)
            if not raw:
                time.sleep(0.002)
                continue
            buf += raw

            while len(buf) >= FRAME_LEN:
                pos = buf.find(TRAILER)
                if pos < 0:
                    buf = buf[-(FRAME_LEN - 1):]
                    break
                frame_start = pos - 13
                if frame_start < 0:
                    buf = buf[pos + len(TRAILER):]
                    continue
                frame = buf[frame_start:frame_start + FRAME_LEN]
                buf = buf[frame_start + FRAME_LEN:]

                frame_count += 1

                # 方法1: 标准SBUS, 使用 byte[1:23] (22 bytes, 跳过header byte 0)
                payload_22 = frame[1:23]
                ch16 = extract_channels_sbus(payload_22)

                # 方法2: 备选, 使用 byte[2:13] (11 bytes, 跳过2字节header)
                payload_11 = frame[2:13]
                ch8 = extract_channels_alt(payload_11)

                # 方法3: 标准SBUS, 使用 byte[2:24] (22 bytes, 跳过2字节header 00 0F)
                payload_22b = frame[2:24]
                ch16b = extract_channels_sbus(payload_22b)

                changed = False
                if last_method1 is None or any(abs(ch16[i] - last_method1[i]) > 5 for i in range(8)):
                    changed = True
                    last_method1 = list(ch16[:8])

                if changed or frame_count <= 5 or frame_count % 50 == 0:
                    # Method 1
                    parts1 = " ".join(f"{ch16[i]:>5}" for i in range(4))
                    sw1 = " ".join(f"{ch16[i]:>5}({label(i,ch16[i])})" for i in range(4, 8))
                    print(f"{frame_count:>6} | {'M1[1:23]':6} | {parts1} | {sw1}")

                    # Method 2
                    parts2 = " ".join(f"{ch8[i]:>5}" for i in range(4))
                    sw2 = " ".join(f"{ch8[i]:>5}({label(i,ch8[i])})" for i in range(4, 8))
                    print(f"{'':>6} | {'M2[2:13]':6} | {parts2} | {sw2}")

                    # Method 3
                    parts3 = " ".join(f"{ch16b[i]:>5}" for i in range(4))
                    sw3 = " ".join(f"{ch16b[i]:>5}({label(i,ch16b[i])})" for i in range(4, 8))
                    print(f"{'':>6} | {'M3[2:24]':6} | {parts3} | {sw3}")
                    print()

    except KeyboardInterrupt:
        print(f"\n[INFO] 共 {frame_count} 帧")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
