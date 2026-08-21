#!/usr/bin/env python3
"""
ET08A W.BUS 遥控器信号测试脚本
================================
协议：25字节帧，100000波特率，W.BUS (兼容S.BUS)
帧格式：[00 0F header][11B payload=8ch×11bit] + [12B fixed trailer 00 04 20 00 01 08 40 00 02 10 00 03]
通道：CH1=副翼 CH2=升降 CH3=油门 CH4=方向
      CH5=SA(2档) CH6=SB(3档) CH7=SC(3档) CH8=SD(2档)
开关值：高=353, 中=1024, 低=1694
SA/SD 瞬态：切换时会出现短暂1024值（无物理卡口）
"""

import serial
import struct
import time
import sys
import os
from collections import deque

# ============ 配置 ============
PORT = "/dev/ttyUSB0"
BAUD = 100000
FRAME_LEN = 25
TRAILER = bytes([0x00, 0x04, 0x20, 0x00, 0x01, 0x08, 0x40, 0x00, 0x02, 0x10, 0x00, 0x03])

CH_NAMES = ["CH1:Roll", "CH2:Pitch", "CH3:Throttle", "CH4:Yaw",
            "CH5:SA(2p)", "CH6:SB(3p)", "CH7:SC(3p)", "CH8:SD(2p)"]

SWITCH_HIGH = 353
SWITCH_MID  = 1024
SWITCH_LOW  = 1694

# 开关档位名称
def switch_label(ch, val):
    """返回开关通道的档位描述"""
    is_2pos = (ch == 4 or ch == 7)  # SA, SD
    if is_2pos:
        if abs(val - SWITCH_HIGH) < 100:
            return "HIGH"
        elif abs(val - SWITCH_LOW) < 100:
            return "LOW "
        else:
            return "TRNS"  # 瞬态, 没有物理卡口
    else:  # 3-pos: SB, SC
        if abs(val - SWITCH_HIGH) < 100:
            return "HIGH"
        elif abs(val - SWITCH_MID) < 100:
            return "MID "
        elif abs(val - SWITCH_LOW) < 100:
            return "LOW "
        else:
            return "????"


def find_frame(data: bytes) -> tuple:
    """
    在字节流中搜索完整25字节帧
    搜索策略：扫描12字节固定trailer,
    验证前13字节中有 header 00 0F
    """
    if len(data) < FRAME_LEN:
        return None, data

    # 搜索 trailer 的起始位置
    for i in range(len(data) - FRAME_LEN + 1):
        candidate = data[i:i + FRAME_LEN]
        if candidate[13:25] == TRAILER:
            header = candidate[0:2]
            if header == b'\x00\x0f':
                return candidate, data[i + FRAME_LEN:]
    # 没找到完整帧，保留最后24字节用于拼接
    if len(data) > FRAME_LEN:
        return None, data[-(FRAME_LEN - 1):]
    return None, data


def decode_channels(frame: bytes) -> list:
    """从25字节帧中解码8通道11-bit值"""
    # 提取 payload 11字节 (byte 2-12)
    payload = frame[2:13]

    # 88 bits → 8 channels × 11 bits
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
    print("=" * 70)
    print("  ET08A W.BUS 遥控器信号测试")
    print("=" * 70)
    print(f"  端口: {PORT}")
    print(f"  波特率: {BAUD}")
    print(f"  帧长度: {FRAME_LEN} bytes")
    print(f"  Trailer: {' '.join(f'{b:02X}' for b in TRAILER)}")
    print("=" * 70)

    # 检查串口是否存在
    if not os.path.exists(PORT):
        print(f"\n[ERROR] 串口 {PORT} 不存在!")
        print(f"  可用串口: {[f for f in os.listdir('/dev') if 'ttyUSB' in f or 'ttyACM' in f]}")
        sys.exit(1)

    # 打开串口
    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUD,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_TWO,  # SBUS/W.BUS 使用2个停止位
            timeout=0.1
        )
    except serial.SerialException as e:
        print(f"\n[ERROR] 无法打开串口: {e}")
        print(f"  尝试: sudo chmod 666 {PORT}")
        sys.exit(1)

    print(f"\n[INFO] 串口已打开, 开始读取数据...")
    print(f"[INFO] 请操作遥控器的摇杆和开关, 观察数值变化")
    print(f"[INFO] 按 Ctrl+C 退出\n")

    buf = b""
    frame_count = 0
    error_count = 0
    last_print_time = time.time()
    last_channels = [0] * 8
    changed_channels = set()

    # 历史记录（用于检测瞬态）
    history = {i: deque(maxlen=5) for i in range(8)}

    # 表头
    header_line = "  ".join(f"{name:>16}" for name in CH_NAMES)
    print(f"  {'Frame#':>6}  {header_line}")
    print("  " + "-" * 150)

    try:
        while True:
            # 阻塞读取
            try:
                raw = ser.read(1024)
            except serial.SerialException:
                time.sleep(0.01)
                continue

            if not raw:
                time.sleep(0.001)
                continue

            buf += raw

            # 处理所有完整帧
            while True:
                frame, buf = find_frame(buf)
                if frame is None:
                    break

                channels = decode_channels(frame)
                frame_count += 1

                # 检测变化
                for i in range(8):
                    history[i].append(channels[i])
                    if channels[i] != last_channels[i]:
                        changed_channels.add(i)

                # 每0.5秒或检测到变化时打印
                now = time.time()
                if changed_channels or (now - last_print_time > 0.5):
                    # 构建彩色输出
                    parts = []
                    for i in range(8):
                        val = channels[i]
                        is_changed = i in changed_channels

                        if i >= 4:  # 开关通道
                            label = switch_label(i, val)
                            s = f"{val:>5}({label})"
                        else:  # 摇杆通道
                            s = f"{val:>5}"

                        if is_changed:
                            s = f"\033[1;33m{s}\033[0m"  # 黄色高亮
                        parts.append(f"{s:>22}")

                    print(f"  {frame_count:>6d}  " + "  ".join(parts))

                    if changed_channels:
                        print(f"        变化通道: {sorted(changed_channels)}")
                        for ch in sorted(changed_channels):
                            hist_vals = list(history[ch])
                            if len(hist_vals) >= 2:
                                # 检测瞬态：值不是稳定端点之一
                                if ch in (4, 7):  # SA, SD 2档
                                    if abs(channels[ch] - SWITCH_MID) < 150:
                                        if abs(hist_vals[-2] - SWITCH_HIGH) < 100 or abs(hist_vals[-2] - SWITCH_LOW) < 100:
                                            print(f"        ⚠ CH{ch+1}({CH_NAMES[ch]}): 检测到瞬态1024! "
                                                  f"上一帧={hist_vals[-2]}, 当前={channels[ch]}")
                                print(f"        CH{ch+1} 历史: {hist_vals}")

                    last_print_time = now
                    changed_channels.clear()

                last_channels = channels[:]

    except KeyboardInterrupt:
        print(f"\n\n[INFO] 用户中断")
    finally:
        ser.close()
        print(f"[INFO] 共接收 {frame_count} 帧, 错误 {error_count} 帧")
        print(f"[INFO] 串口已关闭")


if __name__ == "__main__":
    main()
