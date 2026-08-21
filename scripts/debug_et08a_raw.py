#!/usr/bin/env python3
"""
ET08A W.BUS 原始字节调试脚本
==============================
直接dump 25字节原始帧，用于验证帧格式和通道解码
"""

import serial
import time
import os
import sys

PORT = "/dev/ttyUSB0"
BAUD = 100000
FRAME_LEN = 25
# 已知 trailer (12 bytes)
TRAILER = bytes([0x00, 0x04, 0x20, 0x00, 0x01, 0x08, 0x40, 0x00, 0x02, 0x10, 0x00, 0x03])


def decode_sbus_channels(frame: bytes) -> list:
    """S.BUS/W.BUS 解码: 从 byte[1:23] (22 bytes) 解 16ch x 11bit"""
    payload = frame[1:23]
    all_bits = ""
    for b in payload:
        all_bits += format(b, '08b')
    channels = []
    for i in range(16):
        if (i + 1) * 11 <= len(all_bits):
            val = int(all_bits[i * 11:(i + 1) * 11], 2)
        else:
            val = 0
        channels.append(val)
    return channels


def decode_alt_channels(frame: bytes) -> list:
    """备选解码: 从 byte[2:13] (11 bytes) 解 8ch x 11bit"""
    payload = frame[2:13]
    all_bits = ""
    for b in payload:
        all_bits += format(b, '08b')
    channels = []
    for i in range(8):
        if (i + 1) * 11 <= len(all_bits):
            val = int(all_bits[i * 11:(i + 1) * 11], 2)
        else:
            val = 0
        channels.append(val)
    return channels


def main():
    print("=" * 80)
    print("  ET08A W.BUS 原始帧调试")
    print("=" * 80)

    if not os.path.exists(PORT):
        print(f"[ERROR] {PORT} 不存在!")
        sys.exit(1)

    ser = serial.Serial(PORT, BAUD, timeout=0.1)

    # STM32/CH340 在初始阶段可能有垃圾数据，先清空
    print("[INFO] 清空串口缓冲区...")
    time.sleep(0.5)
    ser.read(4096)

    print("[INFO] 开始采集 20 帧原始数据...\n")

    buf = b""
    frames = []
    dump_count = 0

    while dump_count < 30:
        raw = ser.read(1024)
        if not raw:
            time.sleep(0.002)
            continue
        buf += raw

        # 搜索 trailer
        while len(buf) >= FRAME_LEN:
            pos = buf.find(TRAILER)
            if pos < 0:
                # 没找到 trailer，保留尾部
                buf = buf[-(FRAME_LEN - 1):]
                break

            # trailer 找到了，提取前面的完整帧
            # trailer 在位置 pos，帧起始 = pos - (FRAME_LEN - len(TRAILER))
            frame_start = pos - (FRAME_LEN - len(TRAILER))
            if frame_start < 0:
                # 后面再来
                buf = buf[pos + len(TRAILER):]
                continue

            frame = buf[frame_start:frame_start + FRAME_LEN]
            buf = buf[frame_start + FRAME_LEN:]

            if dump_count < 30:
                dump_count += 1

                # 方法1: SBUS标准解码 (byte 1-22)
                ch16 = decode_sbus_channels(frame)
                # 方法2: 备选解码 (byte 2-13)
                ch8 = decode_alt_channels(frame)

                print(f"--- Frame {dump_count} ---")
                # 原始十六进制
                hex_str = " ".join(f"{b:02X}" for b in frame)
                print(f"  RAW: {hex_str}")

                # 检查头部
                print(f"  Header: {frame[0]:02X} {frame[1]:02X}")
                print(f"  Byte[23] (last): {frame[23]:02X}")
                print(f"  Byte[24] (last): {frame[24]:02X}")

                # SBUS解码 16通道
                print(f"  SBUS(16ch): " + " ".join(f"CH{i+1}={ch16[i]:>5}" for i in range(8)))
                print(f"              " + " ".join(f"CH{i+1}={ch16[i]:>5}" for i in range(8, 16)))

                # 备选解码 8通道
                print(f"  ALT(8ch):   " + " ".join(f"CH{i+1}={ch8[i]:>5}" for i in range(8)))

                # 标志位
                print(f"  SBUS flags: ch17={ch16[16]}, ch18={ch16[17]}, "
                      f"frame_lost={bool(frame[23] & 0x04)}, "
                      f"failsafe={bool(frame[23] & 0x08)}")
                print()

    ser.close()
    print(f"[INFO] 完成, 共采集 {dump_count} 帧")


if __name__ == "__main__":
    main()
