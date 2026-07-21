#!/usr/bin/env python3
"""
诊断脚本：直接通过 CAN 通信查询电机状态

总线电机映射 (来自 params.yaml):
  can0: 腿1 (电机 1/2/3)   → 腿1髋 / 腿1大腿 / 腿1小腿
  can1: 腿2 (电机 4/5/6)   → 腿2髋 / 腿2大腿 / 腿2小腿
  can2: 腿3 (电机 7/8/9)   → 腿3髋 / 腿3大腿 / 腿3小腿
  can3: 腿4 (电机10/11/12) → 腿4髋 / 腿4大腿 / 腿4小腿

用法:
  sudo python3 scripts/check_motor.py                              # 默认全部 12 个电机
  sudo python3 scripts/check_motor.py --iface can0                 # 只查 can0 (腿1)
  sudo python3 scripts/check_motor.py --iface can1                 # 只查 can1 (腿2)
  sudo python3 scripts/check_motor.py --iface can2                 # 只查 can2 (腿3)
  sudo python3 scripts/check_motor.py --iface can3                 # 只查 can3 (腿4)
  sudo python3 scripts/check_motor.py --iface can2 --motor 1       # can2 上第1个 (腿3髋)
  sudo python3 scripts/check_motor.py --iface can2 --eid 0x02      # can2 上 EID=0x02 (腿3大腿)
  sudo python3 scripts/check_motor.py --mode enable --iface can2   # 使能 can2 全部
  sudo python3 scripts/check_motor.py --mode disable               # 停止全部
  sudo python3 scripts/check_motor.py --iface can1 --motor 3 --mode zero   # 标零 (腿2小腿)
  sudo python3 scripts/check_motor.py --iface can1 --motor 3 --mode set_id --new-id 0x01   # 改 CAN ID
  sudo python3 scripts/check_motor.py --iface can1 --mode scan           # 扫描总线发现电机 EID

旋转至指定位置（使能后锁在目标位置，Ctrl+C 停止）:
  sudo python3 scripts/check_motor.py --iface can3 --motor 3 --mode goto --position 0.5
    → 控制腿4小腿旋转到 0.5 rad (瞬时跳变)
  sudo python3 scripts/check_motor.py --iface can3 --motor 3 --mode goto --position -1.0 --speed 0.5
    → 以 0.5 rad/s 速度匀速旋转到 -1.0 rad
  sudo python3 scripts/check_motor.py --iface can3 --eid 0x02 --mode goto --position 0.0 --kp 10 --kd 1
    → 控制腿4大腿，Kp=10 Kd=1，目标 0 rad

持续监控（被动监听，不干扰电机运行）:
  sudo python3 scripts/check_motor.py --iface can1 --motor 3 --mode monitor
    → 监控腿2小腿，Ctrl+C 停止，自动保存 CSV
  sudo python3 scripts/check_motor.py --iface can1 --motor 3 --mode monitor --duration 120
    → 监控 120 秒后自动停止
  sudo python3 scripts/check_motor.py --iface can1 --motor 3 --mode monitor --rate 100 --output my_log.csv
    → 100ms 记录间隔，指定输出文件

监控模式 CSV 字段:
  timestamp, elapsed_s, pos_u16, vel_u16, torque_u16, temp_u16,
  pos_rad, vel_rad_s, torque_Nm, temp_C, vbus_V,
  fault1, fault2, fault3, anomaly, hb_frames, detail_frames, err_frames
"""
import socket
import struct
import time
import sys
import argparse
import datetime
import csv as csv_module

# ============== CAN 协议常量 ==============
CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF

# 通信类型 (bits 24-28)
COMM_GET_ID              = 0x00
COMM_MOTION_CONTROL      = 0x01
COMM_MOTOR_REQUEST       = 0x02  # 电机反馈帧
COMM_MOTOR_ENABLE        = 0x03
COMM_MOTOR_STOP          = 0x04
COMM_SET_POS_ZERO        = 0x06
COMM_SET_CAN_ID          = 0x07
COMM_GET_SINGLE_PARAM    = 0x11
COMM_SET_SINGLE_PARAM    = 0x12
COMM_ERROR_FEEDBACK      = 0x15

# 参数索引
PARAM_RUN_MODE    = 0x7005
PARAM_IQ_REF      = 0x7006
PARAM_SPD_REF     = 0x700A
PARAM_LIMIT_TORQUE= 0x700B
PARAM_CUR_KP      = 0x7010
PARAM_CUR_KI      = 0x7011
PARAM_LOC_REF     = 0x7016
PARAM_LIMIT_SPD   = 0x7017
PARAM_LIMIT_CUR   = 0x7018
PARAM_MECH_POS    = 0x7019  # 机械角度 (只读)
PARAM_IQF         = 0x701A  # iq 滤波值 (只读)
PARAM_MECH_VEL    = 0x701B  # 机械转速 (只读)
PARAM_VBUS        = 0x701C  # 母线电压 (只读)
PARAM_ROTATION    = 0x701D  # 圈数 (只读)

# RS02 故障寄存器 (只读)
PARAM_FAULT1      = 0x3022  # 故障码 (fault1)
PARAM_FAULT2      = 0x3024  # 驱动芯片故障码1 (fault2)
PARAM_FAULT3      = 0x3025  # 驱动芯片故障码2 (fault3)

# 电机类型参数映射 (actuator_type=2 → ROBSTRIDE_02)
ACTUATOR_PARAMS = {
    0: (4*3.14159, 50, 17, 500, 5),
    1: (4*3.14159, 44, 17, 500, 5),
    2: (4*3.14159, 44, 17, 500, 5),
    3: (4*3.14159, 50, 60, 5000, 100),
    4: (4*3.14159, 15, 120, 5000, 100),
    5: (4*3.14159, 33, 17, 500, 5),
    6: (4*3.14159, 20, 60, 5000, 100),
}

PARAM_NAMES = {
    0x7005: "run_mode",
    0x7006: "iq_ref",
    0x700A: "spd_ref",
    0x700B: "limit_torque",
    0x7010: "cur_kp",
    0x7011: "cur_ki",
    0x7014: "cur_filt_gain",
    0x7016: "loc_ref",
    0x7017: "limit_spd",
    0x7018: "limit_cur",
    0x7019: "mechPos",
    0x701A: "iqf",
    0x701B: "mechVel",
    0x701C: "VBUS",
    0x701D: "rotation",
    0x3022: "fault1",
    0x3024: "fault2",
    0x3025: "fault3",
}

# RS02 故障码位定义 (0x3022 / error_feedback Byte0~3)
FAULT_BITS = {
    16: "A相电流采样过流",
    14: "电机堵转过载",
    9:  "位置初始化故障",
    8:  "硬件识别故障",
    7:  "编码器未标定",
    5:  "C相电流采样过流",
    4:  "B相电流采样过流",
    3:  "过压故障 (>60V)",
    2:  "欠压故障 (<12V)",
    1:  "驱动芯片故障",
    0:  "电机过温 (>135°C)",
}

def decode_fault_bits(fault_val: int) -> list:
    """解码 RS02 故障寄存器值，返回故障描述列表"""
    if fault_val == 0:
        return []
    faults = []
    for bit, desc in sorted(FAULT_BITS.items(), reverse=True):
        if fault_val & (1 << bit):
            faults.append(f"bit{bit}:{desc}")
    return faults

# DRV 芯片故障码 1 位定义 (0x3024)
DRV_FAULT1_BITS = {
    10: "FAULT(总故障)",
    9:  "VDS_OCP(VDS过流)",
    8:  "GDF(栅极驱动故障)",
    7:  "UVLO(欠压锁存)",
    6:  "OTSD(过温关断)",
    5:  "VDS_HA(A相高侧VDS过流)",
    4:  "VDS_LA(A相低侧VDS过流)",
    3:  "VDS_HB(B相高侧VDS过流)",
    2:  "VDS_LB(B相低侧VDS过流)",
    1:  "VDS_HC(C相高侧VDS过流)",
    0:  "VDS_LC(C相低侧VDS过流)",
}

# DRV 芯片故障码 2 位定义 (0x3025)
DRV_FAULT2_BITS = {
    10: "SA_OC(A相采样过流)",
    9:  "SB_OC(B相采样过流)",
    8:  "SC_OC(C相采样过流)",
    7:  "OTW(过温警告)",
    6:  "GDUV(电荷泵欠压)",
    5:  "VGS_HA(A相高侧栅极故障)",
    4:  "VGS_LA(A相低侧栅极故障)",
    3:  "VGS_HB(B相高侧栅极故障)",
    2:  "VGS_LB(B相低侧栅极故障)",
    1:  "VGS_HC(C相高侧栅极故障)",
    0:  "VGS_LC(C相低侧栅极故障)",
}

def decode_drv_fault1_bits(fault_val: int) -> list:
    """解码 DRV 芯片故障码 1 (0x3024)"""
    if fault_val == 0:
        return []
    faults = []
    for bit, desc in sorted(DRV_FAULT1_BITS.items(), reverse=True):
        if fault_val & (1 << bit):
            faults.append(f"bit{bit}:{desc}")
    return faults

def decode_drv_fault2_bits(fault_val: int) -> list:
    """解码 DRV 芯片故障码 2 (0x3025)"""
    if fault_val == 0:
        return []
    faults = []
    for bit, desc in sorted(DRV_FAULT2_BITS.items(), reverse=True):
        if fault_val & (1 << bit):
            faults.append(f"bit{bit}:{desc}")
    return faults

# ============== SocketCAN 封装 ==============
class MotorCAN:
    def __init__(self, iface: str, master_id: int, device_eid: int, actuator_type: int = 2):
        self.iface = iface
        self.master_id = master_id
        self.device_eid = device_eid
        self.actuator_type = actuator_type
        self.sock = None

    def open(self):
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        # 用接口名字符串绑定 (兼容各 Python 版本)
        self.sock.bind((self.iface,))

        # 设置接收过滤器：只接收目标电机的帧
        can_id_filter = (self.device_eid << 8) | CAN_EFF_FLAG
        can_mask = (0xFF << 8) | CAN_EFF_FLAG
        filter_data = struct.pack('II', can_id_filter, can_mask)
        self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filter_data)

        # 接收超时 100ms
        tv = struct.pack('LL', 0, 100000)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVTIMEO, tv)

        # 发送超时 + 缓冲区 (避免 "No buffer space available" 错误)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDTIMEO, tv)
        sendbuf = 1024 * 1024  # 1MB
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, sendbuf)

        print(f"[✓] 已绑定 {self.iface}, 过滤 CAN EID=0x{self.device_eid:02X}")

    def close(self):
        if self.sock:
            self.sock.close()

    def build_can_id(self, comm_type: int) -> int:
        """构建扩展帧 CAN ID"""
        return (comm_type << 24) | (self.master_id << 8) | self.device_eid | CAN_EFF_FLAG

    def send_frame(self, comm_type: int, data: bytes = b'\x00' * 8):
        """发送 CAN 帧 (struct can_frame = 16 bytes: I B 3x 8B)"""
        can_id = self.build_can_id(comm_type)
        data_padded = (data[:8] + b'\x00' * 8)[:8]  # 确保 8 字节
        frame = struct.pack('=IB3x8B', can_id & 0xFFFFFFFF, 8, *data_padded)
        try:
            self.sock.send(frame)
            print(f"  -> 发送: ID=0x{can_id:08X} data={' '.join(f'{b:02X}' for b in data[:8])}")
        except OSError as e:
            if e.errno == 105:  # ENOBUFS
                print(f"  [!] CAN 发送失败: 缓冲区满 (总线可能未配置或无设备应答)")
                print(f"      请先执行: sudo ip link set {self.iface} type can bitrate 1000000")
                print(f"                sudo ip link set {self.iface} up")
            raise

    def recv_frame(self, timeout_ms: int = 200) -> tuple:
        """接收 CAN 帧，返回 (can_id, data_bytes) 或 None"""
        self.sock.settimeout(timeout_ms / 1000.0)
        try:
            raw = self.sock.recv(16)
            can_id, can_dlc = struct.unpack('IB3x', raw[:8])
            data = raw[8:8+can_dlc]
            return (can_id, data)
        except socket.timeout:
            return None
        except Exception as e:
            print(f"  [!] 接收错误: {e}")
            return None

    def parse_status_frame(self, can_id: int, data: bytes):
        """解析电机状态反馈帧 (通信类型 0x02)"""
        comm_type = (can_id & CAN_EFF_MASK) >> 24
        extra_data = (can_id >> 8) & 0xFFFF
        host_id = can_id & 0xFF

        # 故障标志 (RS02 手册: CAN ID bit21~16 → extra_data bit13~8)
        # bit22~23: 模式状态 (0=Reset, 1=Cali, 2=Motor)
        status_mode             = (extra_data >> 14) & 0x03
        status_uncalibrated     = (extra_data >> 13) & 0x01  # bit21: 未标定
        status_stall_overload   = (extra_data >> 12) & 0x01  # bit20: 堵转过载
        status_mag_fault        = (extra_data >> 11) & 0x01  # bit19: 磁编码故障
        status_overtemp         = (extra_data >> 10) & 0x01  # bit18: 过温
        status_phase_current    = (extra_data >> 9) & 0x01   # bit17: 三相电流故障
        status_undervoltage     = (extra_data >> 8) & 0x01   # bit16: 欠压故障
        device_id               = extra_data & 0xFF

        mode_names = {0: 'Reset', 1: 'Cali', 2: 'Motor'}
        mode_str = mode_names.get(status_mode, f'未知({status_mode})')

        pos, vel, torque, temp = 0.0, 0.0, 0.0, 0.0
        if len(data) >= 8:
            pos_u16   = (data[0] << 8) | data[1]
            vel_u16   = (data[2] << 8) | data[3]
            torque_u16= (data[4] << 8) | data[5]
            temp_u16  = (data[6] << 8) | data[7]

            params = ACTUATOR_PARAMS.get(self.actuator_type, ACTUATOR_PARAMS[2])
            pos_range, vel_range, torque_range, kp_range, kd_range = params

            pos    = ((pos_u16 / 32767.0) - 1.0) * pos_range
            vel    = ((vel_u16 / 32767.0) - 1.0) * vel_range
            torque = ((torque_u16 / 32767.0) - 1.0) * torque_range
            temp   = temp_u16 * 0.1

        return {
            'comm_type': comm_type,
            'host_id': host_id,
            'device_id': device_id,
            'mode': status_mode,
            'mode_str': mode_str,
            'uncalibrated': status_uncalibrated,
            'stall_overload': status_stall_overload,
            'mag_fault': status_mag_fault,
            'overtemp': status_overtemp,
            'phase_current_fault': status_phase_current,
            'undervoltage': status_undervoltage,
            'position_rad': round(pos, 4),
            'velocity_rad_s': round(vel, 4),
            'torque_Nm': round(torque, 4),
            'temperature_C': round(temp, 1),
        }

    def parse_param_frame(self, can_id: int, data: bytes):
        """解析参数读取响应帧 (通信类型 0x11)"""
        comm_type = (can_id & CAN_EFF_MASK) >> 24
        if len(data) < 8:
            return None
        index = data[0] | (data[1] << 8)
        value_bytes = data[4:8]
        value_float = struct.unpack('<f', value_bytes)[0]
        value_uint8 = data[4]
        return {
            'comm_type': comm_type,
            'index': f'0x{index:04X}',
            'name': PARAM_NAMES.get(index, 'unknown'),
            'float_value': round(value_float, 4),
            'uint8_value': value_uint8,
        }

    # ---------- 高层操作 ----------

    def enable(self):
        """使能电机"""
        print("[*] 发送使能命令...")
        self.send_frame(COMM_MOTOR_ENABLE)
        time.sleep(0.05)
        return self.recv_and_parse()

    def disable(self, clear_error: bool = False):
        """停止电机"""
        print("[*] 发送停止命令...")
        data = bytes([1 if clear_error else 0] + [0]*7)
        self.send_frame(COMM_MOTOR_STOP, data)
        time.sleep(0.05)

    def set_zero_position(self):
        """设置电机机械零位（标零）

        必须在失能状态下执行。成功后当前位置即为零点。
        电机返回 type 0x02 反馈帧确认。
        """
        print("[*] 发送标零命令...")
        data = bytes([1] + [0] * 7)  # Byte[0]=1
        self.send_frame(COMM_SET_POS_ZERO, data)
        time.sleep(0.05)
        return self.recv_and_parse()

    def set_can_id(self, new_can_id: int):
        """设置电机 CAN EID（通信类型 7）

        Args:
            new_can_id: 新 CAN EID (0x01~0xFE)，设置后重新上电生效。
                        同时只能有一个电机在线，其他电机需断电。
        """
        print(f"[*] 发送设置 CAN_ID 命令: 0x{self.device_eid:02X} → 0x{new_can_id:02X}")
        # 协议: bit23~16 = 新 CAN_ID, bit15~8 = master_id, bit7~0 = 目标电机 EID
        can_id = (COMM_SET_CAN_ID << 24) | (new_can_id << 16) | (self.master_id << 8) | self.device_eid | CAN_EFF_FLAG
        data = b'\x00' * 8
        frame = struct.pack('=IB3x8B', can_id & 0xFFFFFFFF, 8, *data)
        try:
            self.sock.send(frame)
            print(f"  -> 发送: ID=0x{can_id:08X} data={' '.join(f'{b:02X}' for b in data)}")
        except OSError as e:
            if e.errno == 105:
                print(f"  [!] CAN 发送失败: 缓冲区满")
                print(f"      请确认 {self.iface} 已正确配置并 UP，且总线有终端电阻。")
                print(f"      快速检查: ip -details link show {self.iface}")
            return False
        time.sleep(0.1)
        result = self.recv_frame(timeout_ms=300)
        if result:
            can_id_rx, data_rx = result
            comm_type = (can_id_rx & CAN_EFF_MASK) >> 24
            print(f"  <- 收到: ID=0x{can_id_rx:08X} type=0x{comm_type:02X} "
                  f"data={' '.join(f'{b:02X}' for b in data_rx)}")
            return True
        else:
            print(f"  [!] 无响应（可能已成功，重新上电后生效）")
            return True

    def motion_control(self, position=0.0, velocity=0.0, kp=0.5, kd=0.1):
        """运控模式命令 (通信类型 0x01)，电机会回复状态"""
        self._flush()  # 清空残留帧
        params = ACTUATOR_PARAMS.get(self.actuator_type, ACTUATOR_PARAMS[2])
        pos_range, vel_range, torque_range, kp_max, kd_max = params

        def float_to_uint(val, vmin, vmax, bits=16):
            span = vmax - vmin
            return int((val - vmin) / span * ((1 << bits) - 1)) & ((1 << bits) - 1)

        pos_u = float_to_uint(position, -pos_range, pos_range)
        vel_u = float_to_uint(velocity, -vel_range, vel_range)
        kp_u  = float_to_uint(kp, 0, kp_max)
        kd_u  = float_to_uint(kd, 0, kd_max)

        data = bytes([
            (pos_u >> 8) & 0xFF, pos_u & 0xFF,
            (vel_u >> 8) & 0xFF, vel_u & 0xFF,
            (kp_u >> 8) & 0xFF,  kp_u & 0xFF,
            (kd_u >> 8) & 0xFF,  kd_u & 0xFF,
        ])
        self.send_frame(COMM_MOTION_CONTROL, data)
        time.sleep(0.02)
        return self.recv_and_parse()

    def read_param(self, index: int):
        """读取单个参数，最多重试 3 次。
        每次重试前清空缓冲区，收到非参数响应帧时跳过继续等待。"""
        for attempt in range(3):
            self._flush()  # 时间窗口清空残留帧
            data = bytes([
                index & 0xFF,
                (index >> 8) & 0xFF,
                0, 0, 0, 0, 0, 0,
            ])
            self.send_frame(COMM_GET_SINGLE_PARAM, data)

            # 等待响应，跳过残留的状态帧 / 故障帧
            deadline = time.time() + 0.15
            while time.time() < deadline:
                remaining = deadline - time.time()
                result = self.recv_frame(timeout_ms=max(10, int(remaining * 1000)))
                if result is None:
                    break  # 超时
                can_id, raw_data = result
                comm_type = (can_id & CAN_EFF_MASK) >> 24
                if comm_type == COMM_GET_SINGLE_PARAM:
                    # 静默打印（recv_and_parse 不打，这里自己打）
                    print(f"  <- 收到: ID=0x{can_id:08X} type=0x{comm_type:02X} "
                          f"data={' '.join(f'{b:02X}' for b in raw_data)}")
                    return self.parse_param_frame(can_id, raw_data)
                elif comm_type == COMM_ERROR_FEEDBACK:
                    fault_val = int.from_bytes(raw_data[0:4], 'little') if len(raw_data) >= 4 else 0
                    faults = decode_fault_bits(fault_val)
                    if faults:
                        print(f"  [!] 故障反馈: {', '.join(faults)} (跳过)")
                    else:
                        print(f"  [!] 故障反馈: fault=0x{fault_val:08X} (跳过)")
                    continue
                else:
                    # 残留的状态帧等，静默丢弃
                    continue

            if attempt < 2:
                print(f"     (重试 {attempt+2}/3...)")
        return None

    def _flush(self):
        """时间窗口清空接收缓冲区中的残留帧（约 20ms）"""
        self.sock.settimeout(0.003)
        deadline = time.time() + 0.02
        try:
            while time.time() < deadline:
                self.sock.recv(16)
        except (socket.timeout, BlockingIOError, OSError):
            pass
        self.sock.settimeout(0.2)

    def recv_and_parse(self):
        """接收并解析响应帧"""
        result = self.recv_frame(timeout_ms=200)
        if result is None:
            print("  [!] 无响应 (超时)")
            return None

        can_id, data = result
        comm_type = (can_id & CAN_EFF_MASK) >> 24
        print(f"  <- 收到: ID=0x{can_id:08X} type=0x{comm_type:02X} "
              f"data={' '.join(f'{b:02X}' for b in data)}")

        if comm_type == COMM_MOTOR_REQUEST:
            return self.parse_status_frame(can_id, data)
        elif comm_type == COMM_GET_SINGLE_PARAM:
            return self.parse_param_frame(can_id, data)
        elif comm_type == COMM_ERROR_FEEDBACK:
            # RS02 手册: Byte0~3=fault值, Byte4~7=warning值
            fault_val = int.from_bytes(data[0:4], 'little') if len(data) >= 4 else 0
            warn_val = int.from_bytes(data[4:8], 'little') if len(data) >= 8 else 0
            faults = decode_fault_bits(fault_val)
            if faults:
                print(f"  [!] 故障反馈! fault=0x{fault_val:08X}: {', '.join(faults)}")
            else:
                print(f"  [!] 故障反馈! fault=0x{fault_val:08X} (无故障位)")
            return {'comm_type': comm_type, 'fault_val': fault_val, 'warn_val': warn_val,
                    'faults': faults}
        else:
            return {'comm_type': comm_type, 'raw_data': data.hex()}

    def passive_listen(self, duration_ms: int = 600) -> dict:
        """被动监听该电机自发反馈帧 (type 0x02)，不发送任何指令。

        忽略主机发出的运控命令 (type 0x01)，只统计电机自发帧。
        用于检测 MCU 冻结 (反馈帧在发但数据为 0x7FFF)。

        Returns:
            {
                'heartbeat_seen': bool,
                'detail_seen': bool,
                'position_raw': int|None,
                'velocity_raw': int|None,
                'torque_raw': int|None,
                'temp_raw': int|None,
                'total_frames': int,
                'diagnosis': str,
            }
        """
        result = {
            'heartbeat_seen': False,
            'detail_seen': False,
            'position_raw': None,
            'velocity_raw': None,
            'torque_raw': None,
            'temp_raw': None,
            'total_frames': 0,
            'diagnosis': '',
        }

        # 打开临时无过滤 socket
        try:
            raw_sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            raw_sock.bind((self.iface,))
            raw_sock.settimeout(duration_ms / 1000.0)
        except Exception as e:
            print(f"  [!] 无法打开监听 socket: {e}")
            result['diagnosis'] = '无法监听'
            return result

        try:
            deadline = time.time() + duration_ms / 1000.0
            while time.time() < deadline:
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                raw_sock.settimeout(remaining)
                try:
                    raw = raw_sock.recv(16)
                    can_id_raw, can_dlc = struct.unpack('IB3x', raw[:8])
                    data = raw[8:8+can_dlc]

                    can_id_nomask = can_id_raw & CAN_EFF_MASK
                    comm_type = can_id_nomask >> 24
                    low_byte = can_id_nomask & 0xFF
                    extra = (can_id_nomask >> 8) & 0xFFFF

                    # 运控命令帧 (type 0x01): 主机发往电机，不是电机自发帧，忽略
                    # 详细状态帧 (type 0x02): 电机自发反馈，device_id 在 bits 8-15
                    if comm_type == COMM_MOTOR_REQUEST and (extra & 0xFF) == self.device_eid:
                        result['heartbeat_seen'] = True  # type 0x02 即是电机的"心跳"
                        result['detail_seen'] = True
                        result['total_frames'] += 1
                        if len(data) >= 8:
                            result['position_raw'] = (data[0] << 8) | data[1]
                            result['velocity_raw'] = (data[2] << 8) | data[3]
                            result['torque_raw'] = (data[4] << 8) | data[5]
                            result['temp_raw'] = (data[6] << 8) | data[7]

                except socket.timeout:
                    break
                except Exception:
                    continue
        finally:
            raw_sock.close()

        # 生成诊断结论 (heartbeat_seen/detail_seen 现在都来自 type 0x02 反馈帧)
        if result['heartbeat_seen']:
            pos_raw = result['position_raw']
            vel_raw = result['velocity_raw']
            if pos_raw == 0x7FFF and vel_raw == 0x7FFF:
                result['diagnosis'] = (
                    'MCU 冻结: 反馈帧仍在发送但数据异常 (位置=速度=0x7FFF)，'
                    '主循环已挂死，电机保持最后力矩输出（关节可能僵硬）。'
                    '恢复需断电重启。'
                )
            else:
                result['diagnosis'] = '通信正常 (收到电机反馈帧)'
        else:
            result['diagnosis'] = (
                '完全无响应: 可能未上电、CAN 接线断开、或 CAN 收发器损坏'
            )

        return result


# ============== 总线电机映射 ==============
# 每个 CAN 总线上有 3 个电机，CAN EID 固定为 0x01, 0x02, 0x03
CAN_EIDS = [0x01, 0x02, 0x03]

# 每条 CAN 总线对应一条腿，EID 0x01=髋, 0x02=大腿, 0x03=小腿
JOINT_NAMES = {
    'can0': {0x01: '腿1髋', 0x02: '腿1大腿', 0x03: '腿1小腿'},
    'can1': {0x01: '腿2髋', 0x02: '腿2大腿', 0x03: '腿2小腿'},
    'can2': {0x01: '腿3髋', 0x02: '腿3大腿', 0x03: '腿3小腿'},
    'can3': {0x01: '腿4髋', 0x02: '腿4大腿', 0x03: '腿4小腿'},
}

# 每个 CAN 总线起始全局 ID
BUS_START_ID = {'can0': 1, 'can1': 4, 'can2': 7, 'can3': 10, 'can4': 13}


def motor_label(iface: str, eid: int) -> str:
    """生成电机标签，优先用已知关节名"""
    name = JOINT_NAMES.get(iface, {}).get(eid)
    global_id = BUS_START_ID.get(iface, 0) + CAN_EIDS.index(eid)
    if name:
        return f"{name} (全局ID={global_id}, EID=0x{eid:02X})"
    return f"电机{global_id} (EID=0x{eid:02X})"


def safe_fmt(val, fmt='>10.4f'):
    """安全格式化浮点数，None 或非数字时返回 N/A"""
    if val is None or not isinstance(val, (int, float)):
        return '       N/A'
    return f'{val:{fmt}}'


def motor_short_name(iface: str, eid: int) -> str:
    """简短标签，用于汇总表头"""
    name = JOINT_NAMES.get(iface, {}).get(eid)
    if name:
        return name
    global_id = BUS_START_ID.get(iface, 0) + CAN_EIDS.index(eid)
    return f"电机{global_id}"

def diagnose_one(iface, master_id, actuator_type, can_eid, label):
    """诊断单个电机，返回状态字典"""
    print(f"\n{'─'*50}")
    print(f"  {label}")
    print(f"{'─'*50}")

    motor = MotorCAN(iface, master_id, can_eid, actuator_type)
    try:
        motor.open()

        # 1. 读运行模式
        print("  [1] 运行模式:", end=" ")
        result = motor.read_param(PARAM_RUN_MODE)
        mode = None
        if result:
            mode_names = {0: '运控', 1: '位置PP', 2: '速度', 3: '电流', 4: '零点', 5: '位置CSP'}
            mode = result.get('uint8_value', None)
            print(f"{mode} ({mode_names.get(mode, '未知')})")

        # 2. 读关键参数
        print("  [2] 关键参数:")
        params_to_read = [
            (PARAM_VBUS,     '母线电压'),
            (PARAM_MECH_POS, '机械角度'),
            (PARAM_MECH_VEL, '机械转速'),
            (PARAM_IQF,      'iq 滤波'),
            (PARAM_ROTATION, '圈数'),
        ]
        for idx, label_p in params_to_read:
            result = motor.read_param(idx)
            val = result.get('float_value') if result else None
            if val is not None:
                print(f"      {label_p:8s}: {val:>10.4f}")
            else:
                print(f"      {label_p:8s}: {'N/A':>10s}")
            time.sleep(0.01)

        # 3. 运控零指令获取状态帧
        print("  [3] 实时状态 (零指令):")
        result = motor.motion_control(position=0, velocity=0, kp=0, kd=0)
        status = {}
        if result:
            status = result
            mode_str = result.get('mode_str', '?')
            print(f"      模式:   {mode_str}")
            print(f"      位置:   {safe_fmt(result.get('position_rad'))} rad")
            print(f"      速度:   {safe_fmt(result.get('velocity_rad_s'))} rad/s")
            print(f"      力矩:   {safe_fmt(result.get('torque_Nm'))} Nm")
            print(f"      温度:   {safe_fmt(result.get('temperature_C'), '>10.1f')} °C")
            flags = []
            if result.get('uncalibrated'):       flags.append('未标定')
            if result.get('stall_overload'):     flags.append('堵转过载')
            if result.get('mag_fault'):          flags.append('磁编码故障')
            if result.get('overtemp'):           flags.append('过温')
            if result.get('phase_current_fault'):flags.append('相电流故障')
            if result.get('undervoltage'):       flags.append('欠压')
            if flags:
                print(f"      ⚠ 状态帧故障: {', '.join(flags)}")
            else:
                print(f"      ✓ 状态帧无故障")
        else:
            status = {}

        # 3b. 读故障寄存器 (0x3022/0x3024/0x3025)
        print("  [3b] 故障寄存器:")
        fault_regs = [
            (PARAM_FAULT1, 'fault1 (0x3022)', decode_fault_bits),
            (PARAM_FAULT2, 'fault2 (0x3024)', decode_drv_fault1_bits),
            (PARAM_FAULT3, 'fault3 (0x3025)', decode_drv_fault2_bits),
        ]
        all_faults = []
        for reg_idx, reg_name, decoder in fault_regs:
            result = motor.read_param(reg_idx)
            if result:
                raw_val = int(result.get('float_value', 0))
                faults = decoder(raw_val)
                if faults:
                    print(f"      {reg_name}: 0x{raw_val:08X} → {', '.join(faults)}")
                    all_faults.extend(faults)
                else:
                    print(f"      {reg_name}: 0x{raw_val:08X} (无故障)")
                status[f'_{reg_name}'] = raw_val
                status['_faults'] = all_faults
            else:
                print(f"      {reg_name}: N/A")
            time.sleep(0.01)

        # 4. 被动监听 (检测 MCU 冻结 / 半挂状态)
        print("  [4] 被动监听 (检测 MCU 冻结):")
        diag = motor.passive_listen(duration_ms=600)
        total = diag['total_frames']
        hb = diag['heartbeat_seen']
        dt = diag['detail_seen']

        print(f"      收到自发帧: {total} 帧  (反馈帧={'✓' if hb else '✗'})")

        if hb:
            pos_raw = diag.get('position_raw')
            vel_raw = diag.get('velocity_raw')
            torque_raw = diag.get('torque_raw')
            temp_raw = diag.get('temp_raw')
            print(f"      反馈帧数据: pos=0x{pos_raw:04X} vel=0x{vel_raw:04X} "
                  f"torque=0x{torque_raw:04X} temp=0x{temp_raw:04X}")

        diagnosis = diag['diagnosis']
        if '冻结' in diagnosis:
            print(f"      ⚠ {diagnosis}")
        elif '部分异常' in diagnosis:
            print(f"      ⚠ {diagnosis}")
        elif '完全无响应' in diagnosis:
            print(f"      ✗ {diagnosis}")
        elif '异常' in diagnosis:
            print(f"      ⚠ {diagnosis}")
        else:
            print(f"      {diagnosis}")

        # 将冻结检测结果附加到状态中
        status['_diag'] = diag
        return status
    finally:
        motor.close()


def monitor_motor(iface: str, master_id: int, actuator_type: int,
                  can_eid: int, label: str, duration_s: float,
                  csv_path: str, log_interval_ms: int = 50):
    """持续被动监听电机心跳帧，记录到 CSV 文件。

    监听心跳帧 (type 0x01) 和详细状态帧 (type 0x02)，
    定期读取故障寄存器和母线电压。
    检测异常 (vel=0x7FFF, temp=0xFFFF, ERROR_FEEDBACK) 时高亮打印。

    Args:
        duration_s: 监控时长 (0=无限)
        csv_path: CSV 输出路径
        log_interval_ms: CSV 记录间隔 (ms)
    """
    params = ACTUATOR_PARAMS.get(actuator_type, ACTUATOR_PARAMS[2])
    pos_range, vel_range, torque_range, kp_range, kd_range = params

    # 打开 CSV
    csv_file = open(csv_path, 'w', newline='')
    csv_writer = csv_module.writer(csv_file)
    csv_writer.writerow([
        'timestamp', 'elapsed_s',
        'pos_u16', 'vel_u16', 'torque_u16', 'temp_u16',
        'pos_rad', 'vel_rad_s', 'torque_Nm', 'temp_C',
        'vbus_V', 'fault1', 'fault2', 'fault3',
        'anomaly', 'hb_frames', 'detail_frames', 'err_frames'
    ])
    csv_file.flush()

    # 打开无过滤监听 socket
    raw_sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    raw_sock.bind((iface,))
    raw_sock.settimeout(0.01)

    # 用于读取故障寄存器的 socket（有过滤）
    diag_motor = MotorCAN(iface, master_id, can_eid, actuator_type)
    diag_motor.open()

    print(f"\n{'='*60}")
    print(f"  持续监控: {label}")
    print(f"  CAN: {iface}  EID: 0x{can_eid:02X}")
    print(f"  日志: {csv_path}")
    print(f"  记录间隔: {log_interval_ms}ms")
    if duration_s > 0:
        print(f"  时长: {duration_s:.0f}s")
    else:
        print(f"  时长: 无限 (Ctrl+C 停止)")
    print(f"{'='*60}")
    print(f"  {'时间':>12s} {'位置(rad)':>9s} {'速度(r/s)':>9s} {'力矩(Nm)':>8s} {'温度°C':>7s} {'电压V':>7s} {'故障':s}")
    print(f"  {'─'*12} {'─'*9} {'─'*9} {'─'*8} {'─'*7} {'─'*7} {'─'*20}")

    start_time = time.time()
    t0 = time.time()
    last_log = t0
    last_fault_read = t0
    last_print = t0

    # 累计值
    pos_u16 = 0x7FFF
    vel_u16 = 0x7FFF
    torque_u16 = 0
    temp_u16 = 0xFFFF
    vbus = None
    fault1 = fault2 = fault3 = 0
    hb_count = 0
    detail_count = 0
    err_count = 0
    err_bits = {}  # bit → count
    last_err_print = 0
    anomaly = ''
    prev_anomaly = ''

    def convert(u16, rng):
        if u16 == 0x7FFF:
            return None
        return round(((u16 / 32767.0) - 1.0) * rng, 4)

    anomaly_debounce = {'state': '', 'since': 0.0}
    DEBOUNCE_MS = 0.3  # 异常状态需持续 300ms 才触发打印

    try:
        while True:
            now = time.time()
            if duration_s > 0 and (now - start_time) > duration_s:
                break

            # 接收帧
            try:
                raw = raw_sock.recv(16)
                can_id_raw, can_dlc = struct.unpack('IB3x', raw[:8])
                data = raw[8:8+can_dlc]
                can_id_nomask = can_id_raw & CAN_EFF_MASK
                comm_type = can_id_nomask >> 24
                low_byte = can_id_nomask & 0xFF
                extra = (can_id_nomask >> 8) & 0xFFFF

                # 运控命令帧 (type 0x01): 主机发往电机，忽略
                # 电机反馈帧 (type 0x02): 电机自发状态，这是唯一可靠的电机数据来源
                if comm_type == COMM_MOTOR_REQUEST and (extra & 0xFF) == can_eid:
                    hb_count += 1
                    detail_count += 1
                    if len(data) >= 8:
                        pos_u16   = (data[0] << 8) | data[1]
                        vel_u16   = (data[2] << 8) | data[3]
                        torque_u16= (data[4] << 8) | data[5]
                        temp_u16  = (data[6] << 8) | data[7]

                # 故障反馈帧
                if comm_type == COMM_ERROR_FEEDBACK:
                    err_count += 1
                    fault_val = int.from_bytes(data[0:4], 'little') if len(data) >= 4 else 0
                    # 解码并统计故障位
                    for bit, desc in FAULT_BITS.items():
                        if fault_val & (1 << bit):
                            err_bits[bit] = err_bits.get(bit, 0) + 1
                    # 每 3s 打印一次故障位统计
                    if err_count > 0 and (now - last_err_print) > 3.0:
                        last_err_print = now
                        parts = [f"{FAULT_BITS[b]}:{err_bits[b]}" for b in sorted(err_bits.keys())]
                        detail = ", ".join(parts)
                        print(f"\n  [故障帧解码] 累计故障:[{detail}] (共{err_count}帧)")
                        print(f"     最新: fault=0x{fault_val:08X}")
                        print(f"     原始: pos=0x{pos_u16:04X} vel=0x{vel_u16:04X} "
                              f"torque=0x{torque_u16:04X} temp=0x{temp_u16:04X}")

            except socket.timeout:
                pass
            except OSError as e:
                print(f"\n  [!] CAN 接口错误: {e}")
                print(f"  [*] 监控结束")
                break

            # 定期读故障寄存器 (每 1s)
            if now - last_fault_read > 1.0:
                last_fault_read = now
                for reg_idx, store in [(PARAM_FAULT1, 'f1'), (PARAM_FAULT2, 'f2'), (PARAM_FAULT3, 'f3')]:
                    r = diag_motor.read_param(reg_idx)
                    if r:
                        val = int(r.get('float_value', 0))
                        if store == 'f1': fault1 = val
                        elif store == 'f2': fault2 = val
                        else: fault3 = val
                # 读母线电压
                r = diag_motor.read_param(PARAM_VBUS)
                if r:
                    vbus = r.get('float_value')

            # 判断异常（仅用可靠指标，心跳 vel=0x7FFF/temp=0xFFFF 是 RS02 正常行为）
            anomaly = ''
            if err_count > hb_count * 0.1 and hb_count > 10:
                anomaly += 'ERR_FLOOD '
            if fault1 or fault2 or fault3:
                # 解码 DRV 故障
                parts = []
                if fault1:
                    f1 = decode_fault_bits(fault1)
                    if f1: parts.append('f1:' + '|'.join(f1))
                    else: parts.append(f'f1=0x{fault1:08X}')
                if fault2:
                    f2 = decode_drv_fault1_bits(fault2)
                    if f2: parts.append('f2:' + '|'.join(f2))
                    else: parts.append(f'f2=0x{fault2:08X}')
                if fault3:
                    f3 = decode_drv_fault2_bits(fault3)
                    if f3: parts.append('f3:' + '|'.join(f3))
                    else: parts.append(f'f3=0x{fault3:08X}')
                anomaly += '|'.join(parts) + ' '
            anomaly = anomaly.strip()

            # 防抖：异常状态需持续 DEBOUNCE_MS 才触发打印
            if anomaly != anomaly_debounce['state']:
                anomaly_debounce['state'] = anomaly
                anomaly_debounce['since'] = now
            elif anomaly != prev_anomaly and (now - anomaly_debounce['since']) > DEBOUNCE_MS:
                # 状态变化已持续足够久，触发打印
                if anomaly:
                    print(f"\n  ⚠ [{now - start_time:7.2f}s] 异常: {anomaly}")
                    print(f"     原始: pos=0x{pos_u16:04X} vel=0x{vel_u16:04X} "
                          f"torque=0x{torque_u16:04X} temp=0x{temp_u16:04X}")
                    # 异常确认后记录一行到 CSV
                    elapsed = now - start_time
                    ts = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
                    pos_r = convert(pos_u16, pos_range)
                    vel_r = convert(vel_u16, vel_range)
                    trq_r = convert(torque_u16, torque_range)
                    tmp_r = None if temp_u16 == 0xFFFF else round(temp_u16 * 0.1, 1)
                    csv_writer.writerow([
                        ts, f'{elapsed:.3f}',
                        f'0x{pos_u16:04X}', f'0x{vel_u16:04X}',
                        f'0x{torque_u16:04X}', f'0x{temp_u16:04X}',
                        f'{pos_r:.4f}' if pos_r is not None else 'N/A',
                        f'{vel_r:.4f}' if vel_r is not None else 'N/A',
                        f'{trq_r:.4f}' if trq_r is not None else 'N/A',
                        f'{tmp_r:.1f}' if tmp_r is not None else 'N/A',
                        f'{vbus:.2f}' if vbus is not None else 'N/A',
                        f'0x{fault1:08X}', f'0x{fault2:08X}', f'0x{fault3:08X}',
                        anomaly, hb_count, detail_count, err_count
                    ])
                    csv_file.flush()
                else:
                    print(f"\n  ✓ [{now - start_time:7.2f}s] 异常恢复")
                prev_anomaly = anomaly

            # 定期打印状态 (每 1s)
            if now - last_print > 1.0:
                last_print = now
                elapsed = now - start_time
                pos_r = convert(pos_u16, pos_range)
                vel_r = convert(vel_u16, vel_range)
                trq_r = convert(torque_u16, torque_range)
                tmp_r = None if temp_u16 == 0xFFFF else round(temp_u16 * 0.1, 1)
                vbus_str = f'{vbus:7.2f}' if vbus is not None else '    N/A'
                status_str = anomaly if anomaly else 'OK'
                print(f"  {elapsed:9.2f}s  "
                      f"{pos_r if pos_r is not None else 'N/A':>9} "
                      f"{vel_r if vel_r is not None else 'N/A':>9} "
                      f"{trq_r if trq_r is not None else 'N/A':>8} "
                      f"{tmp_r if tmp_r is not None else 'N/A':>7} "
                      f"{vbus_str:>7s} "
                      f"{status_str}")

            # 定期写 CSV (每 log_interval_ms)
            if (now - last_log) > (log_interval_ms / 1000.0):
                last_log = now
                elapsed = now - start_time
                ts = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
                pos_r = convert(pos_u16, pos_range)
                vel_r = convert(vel_u16, vel_range)
                trq_r = convert(torque_u16, torque_range)
                tmp_r = None if temp_u16 == 0xFFFF else round(temp_u16 * 0.1, 1)
                csv_writer.writerow([
                    ts, f'{elapsed:.3f}',
                    f'0x{pos_u16:04X}', f'0x{vel_u16:04X}',
                    f'0x{torque_u16:04X}', f'0x{temp_u16:04X}',
                    f'{pos_r:.4f}' if pos_r is not None else 'N/A',
                    f'{vel_r:.4f}' if vel_r is not None else 'N/A',
                    f'{trq_r:.4f}' if trq_r is not None else 'N/A',
                    f'{tmp_r:.1f}' if tmp_r is not None else 'N/A',
                    f'{vbus:.2f}' if vbus is not None else 'N/A',
                    f'0x{fault1:08X}', f'0x{fault2:08X}', f'0x{fault3:08X}',
                    anomaly, hb_count, detail_count, err_count
                ])
                csv_file.flush()

            # 小延迟避免 CPU 空转
            time.sleep(0.001)

    except KeyboardInterrupt:
        print(f"\n  [*] 用户中断")

    finally:
        diag_motor.close()
        raw_sock.close()
        csv_file.close()

    elapsed = time.time() - start_time
    print(f"\n  [✓] 监控结束，共 {elapsed:.1f}s")
    print(f"  反馈帧: {hb_count}  故障反馈帧: {err_count}")
    if err_bits:
        print(f"  故障位统计:")
        for bit in sorted(err_bits.keys()):
            print(f"    bit{bit}: {FAULT_BITS.get(bit, '未知')} → {err_bits[bit]} 次")
    print(f"  日志已保存: {csv_path}")


def main():
    parser = argparse.ArgumentParser(description='诊断 CAN 总线上的电机')
    parser.add_argument('--iface', default='all',
                        help='CAN 接口名 (can0~can4 / all=全部)')
    parser.add_argument('--joint', choices=['hip', 'thigh', 'calf'], default=None,
                        help='关节类型 (髋/大腿/小腿, 需配合 --iface 单总线使用)')
    parser.add_argument('--eid', type=lambda x: int(x, 0), default=None,
                        help='CAN EID (如 0x01/0x02/0x03)')
    parser.add_argument('--motor', type=int, default=None,
                        help='总线上第几个电机 (1/2/3)')
    parser.add_argument('--master-id', type=int, default=0xFF, help='主机 ID')
    parser.add_argument('--actuator-type', type=int, default=2, help='电机类型')
    parser.add_argument('--mode', choices=['status', 'enable', 'disable', 'zero', 'set_id', 'scan', 'all', 'monitor', 'goto'],
                        default='all', help='操作模式 (默认: all=仅查询, monitor=持续监控, scan=扫描EID, zero=标零, set_id=改CANID, goto=旋转至指定位置)')
    parser.add_argument('--duration', type=float, default=0,
                        help='监控时长 (秒, 0=无限, 仅 monitor 模式)')
    parser.add_argument('--output', type=str, default=None,
                        help='监控 CSV 输出路径 (默认: monitor_<时间戳>.csv)')
    parser.add_argument('--rate', type=int, default=50,
                        help='CSV 记录间隔 (ms, 默认 50ms=20Hz, 仅 monitor 模式)')
    parser.add_argument('--new-id', type=lambda x: int(x, 0), default=None,
                        help='新 CAN EID (仅 --mode set_id, 如 0x03)')
    parser.add_argument('--position', type=float, default=0.0,
                        help='目标位置 rad (仅 --mode goto, 如 0.5)')
    parser.add_argument('--kp', type=float, default=5.0,
                        help='位置刚度 (仅 --mode goto, 默认 5.0)')
    parser.add_argument('--kd', type=float, default=0.5,
                        help='速度阻尼 (仅 --mode goto, 默认 0.5)')
    parser.add_argument('--speed', type=float, default=0,
                        help='旋转速度 rad/s (仅 --mode goto, 0=瞬时跳变, 如 0.5)')
    args = parser.parse_args()

    # 确定要扫描的 CAN 总线列表
    if args.iface == 'all':
        buses = ['can0', 'can1', 'can2', 'can3']
    else:
        buses = [args.iface]

    # 构建目标列表：(iface, eid, label, short_name)
    targets = []
    for iface in buses:
        if args.joint is not None:
            joint_to_eid = {'hip': 0x01, 'thigh': 0x02, 'calf': 0x03}
            eid = joint_to_eid.get(args.joint)
            if eid is None:
                print(f"[!] 未知关节: {args.joint}，可选: hip/thigh/calf")
                sys.exit(1)
            eids = [eid]
        elif args.eid is not None:
            eids = [args.eid]
        elif args.motor is not None:
            if args.motor < 1 or args.motor > 3:
                print(f"[!] --motor 取值范围: 1/2/3")
                sys.exit(1)
            eids = [CAN_EIDS[args.motor - 1]]
        else:
            eids = CAN_EIDS

        for eid in eids:
            targets.append({
                'iface': iface,
                'name': motor_short_name(iface, eid),
                'label': motor_label(iface, eid),
                'can_eid': eid,
            })

    print(f"{'='*50}")
    print(f"  电机诊断  (共 {len(targets)} 个)")
    print(f"{'='*50}")

    if args.mode == 'monitor':
        # 监控模式：需要指定单总线 + 单电机
        if args.iface == 'all':
            print("[!] monitor 模式需要指定 --iface canX")
            sys.exit(1)
        if len(targets) > 1:
            print(f"[!] monitor 模式只能监控 1 个电机，当前匹配 {len(targets)} 个，请加 --motor 或 --eid")
            sys.exit(1)

        t = targets[0]
        csv_path = args.output
        if csv_path is None:
            ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            csv_path = f"monitor_{t['name']}_{ts}.csv"

        monitor_motor(
            iface=t['iface'],
            master_id=args.master_id,
            actuator_type=args.actuator_type,
            can_eid=t['can_eid'],
            label=t['label'],
            duration_s=args.duration,
            csv_path=csv_path,
            log_interval_ms=args.rate,
        )
        return

    if args.mode in ('disable',):
        # 停止模式
        for t in targets:
            print(f"\n  停止: {t['label']}")
            motor = MotorCAN(t['iface'], args.master_id, t['can_eid'], args.actuator_type)
            try:
                motor.open()
                motor.disable(clear_error=True)
            finally:
                motor.close()
        print("\n[✓] 全部已发送停止指令")
        return

    if args.mode in ('scan',):
        # 扫描模式：被动监听总线，发现所有在线电机的 EID
        if args.iface == 'all':
            print("[!] scan 模式需要指定 --iface canX")
            sys.exit(1)

        scan_duration = 3.0  # 监听 3 秒
        print(f"\n{'='*60}")
        print(f"  扫描 {args.iface} 总线上的电机 (监听 {scan_duration:.0f}s)...")
        print(f"  注意: 只统计电机自发帧 (type 0x02/0x15)，忽略主机运控命令 (type 0x01)")
        print(f"{'='*60}")

        raw_sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        raw_sock.bind((args.iface,))
        raw_sock.settimeout(0.02)

        # 收集发现的 EID 及统计信息
        found = {}  # eid -> {'hb': N, 'detail': N, 'err': N, 'pos_u16': X}

        deadline = time.time() + scan_duration
        try:
            while time.time() < deadline:
                try:
                    raw = raw_sock.recv(16)
                    can_id_raw, can_dlc = struct.unpack('IB3x', raw[:8])
                    data = raw[8:8+can_dlc]
                    can_id_nomask = can_id_raw & CAN_EFF_MASK
                    comm_type = can_id_nomask >> 24
                    low_byte = can_id_nomask & 0xFF
                    extra = (can_id_nomask >> 8) & 0xFFFF

                    # 运控命令帧 (type 0x01): 主机发往电机，不能作为电机在线的依据
                    # 仅统计作为调试参考，但不列入"发现"列表
                    if comm_type == 0x01:
                        pass  # 忽略，这是主机发出的运控命令

                    # 详细状态/反馈帧 (type 0x02): 来源电机=EID (bits 8-15)，电机自发，可靠
                    if comm_type == 0x02:
                        eid = extra & 0xFF
                        if eid not in found:
                            found[eid] = {'hb': 0, 'detail': 0, 'err': 0, 'pos_u16': 0x7FFF}
                        found[eid]['detail'] += 1
                        if len(data) >= 8:
                            found[eid]['pos_u16'] = (data[0] << 8) | data[1]

                    # 故障反馈帧 (type 0x15): bit15~8 = 电机 EID
                    if comm_type == COMM_ERROR_FEEDBACK:
                        eid = (can_id_nomask >> 8) & 0xFF  # bits 15-8
                        if eid not in found:
                            found[eid] = {'hb': 0, 'detail': 0, 'err': 0, 'pos_u16': 0x7FFF}
                        found[eid]['err'] += 1

                except socket.timeout:
                    pass
        finally:
            raw_sock.close()

        if not found:
            print(f"\n  ✗ 未发现任何电机，请确认: ")
            print(f"     1. {args.iface} 接口已 up (sudo ip link set {args.iface} up)")
            print(f"     2. 电机已上电")
            print(f"     3. 接线正确")
        else:
            print(f"\n  发现 {len(found)} 个电机:")
            print(f"  {'EID':>5s} {'反馈帧':>8s} {'故障帧':>8s} {'最后位置':>10s} {'状态'}")
            print(f"  {'─'*5} {'─'*8} {'─'*8} {'─'*10} {'─'*10}")
            for eid in sorted(found.keys()):
                info = found[eid]
                pos_u = info['pos_u16']
                pos_str = f"0x{pos_u:04X}" if pos_u != 0x7FFF else "-"
                print(f"  0x{eid:02X}  {info['detail']:>8d} {info['err']:>8d} {pos_str:>10s}  在线")

            print(f"\n  对应关节 (假设映射):")
            for eid in sorted(found.keys()):
                name = JOINT_NAMES.get(args.iface, {}).get(eid, f"电机(EID=0x{eid:02X})")
                print(f"    EID=0x{eid:02X} → {name}")

            print(f"\n  用法: sudo python3 scripts/check_motor.py --iface {args.iface} --eid 0x{min(found.keys()):02X}")
        return

    if args.mode in ('zero',):
        # 标零模式：需要失能状态下执行
        for t in targets:
            print(f"\n  标零: {t['label']}")
            motor = MotorCAN(t['iface'], args.master_id, t['can_eid'], args.actuator_type)
            try:
                motor.open()
                result = motor.set_zero_position()
                if result:
                    print(f"    新零点: {safe_fmt(result.get('position_rad'))} rad")
            finally:
                motor.close()
        print("\n[✓] 全部已发送标零指令")
        return

    if args.mode in ('set_id',):
        # 改 CAN ID 模式：只能操作单个电机
        if args.new_id is None:
            print("[!] set_id 模式需要 --new-id 参数 (如 --new-id 0x03)")
            sys.exit(1)
        if args.iface == 'all':
            print("[!] set_id 模式需要指定 --iface canX")
            sys.exit(1)
        if len(targets) > 1:
            print(f"[!] 只能改 1 个电机的 ID，请加 --motor 或 --eid 限定")
            sys.exit(1)
        if not (0x01 <= args.new_id <= 0xFE):
            print(f"[!] 新 ID 范围: 0x01~0xFE")
            sys.exit(1)

        t = targets[0]
        print(f"\n  ⚠ 改 ID: {t['label']}")
        print(f"     从 0x{t['can_eid']:02X} → 0x{args.new_id:02X}")
        print(f"     注意: 该总线上只能有这一个电机在线，其他需断电！")
        print(f"     改完后重新上电生效。")
        response = input(f"     确认? (y/N): ")
        if response.lower() != 'y':
            print("  [*] 取消")
            sys.exit(0)

        motor = MotorCAN(t['iface'], args.master_id, t['can_eid'], args.actuator_type)
        try:
            motor.open()
            motor.set_can_id(args.new_id)
            print(f"\n  [✓] 已发送改 ID 命令，重新上电后生效")
        finally:
            motor.close()
        return

    if args.mode in ('enable',):
        for t in targets:
            print(f"\n  使能: {t['label']}")
            motor = MotorCAN(t['iface'], args.master_id, t['can_eid'], args.actuator_type)
            try:
                motor.open()
                result = motor.enable()
                if result:
                    print(f"    结果: {result}")
            finally:
                motor.close()
        return

    if args.mode in ('goto',):
        # 旋转至指定位置：运控模式，保持目标位置直到 Ctrl+C
        if args.iface == 'all':
            print("[!] goto 模式需要指定 --iface canX")
            sys.exit(1)
        if len(targets) > 1:
            print(f"[!] goto 模式只能控制 1 个电机，当前匹配 {len(targets)} 个，请加 --motor 或 --eid")
            sys.exit(1)

        t = targets[0]
        ramp = args.speed > 0
        print(f"\n  🎯 旋转至目标位置: {t['label']}")
        print(f"     目标: {args.position:.4f} rad")
        if ramp:
            print(f"     速度: {args.speed:.3f} rad/s (斜坡模式)")
        else:
            print(f"     速度: 瞬时 (阶跃模式)")
        print(f"     KP: {args.kp:.1f}  KD: {args.kd:.1f}")
        print(f"     Ctrl+C 停止")

        motor = MotorCAN(t['iface'], args.master_id, t['can_eid'], args.actuator_type)
        try:
            motor.open()
            motor.enable()
            time.sleep(0.1)

            # 先用目标 Kp/Kd 发送零位置命令，让电机在刚度下稳定，读取实际归宿位置
            # 注意：kp=0 时电机自由转动，读到的位置和 kp>0 锁住后的位置不一致
            current_pos = 0.0
            for _ in range(5):
                result = motor.motion_control(position=0.0, velocity=0.0, kp=args.kp, kd=args.kd)
                if result and result.get('position_rad') is not None:
                    current_pos = result['position_rad']
                time.sleep(0.03)
            # 现在 current_pos 是电机在 kp 刚度下能稳定到的实际位置

            start_pos = current_pos
            target_pos = args.position
            if ramp:
                total_dist = abs(target_pos - start_pos)
                ramp_time = total_dist / args.speed
                print(f"     当前: {start_pos:.4f} rad, 预计耗时: {ramp_time:.2f}s")
            else:
                print(f"     当前: {start_pos:.4f} rad")

            print(f"  {'时间':>9s} {'目标(rad)':>10s} {'实际(rad)':>10s} {'速度(r/s)':>12s} {'力矩(Nm)':>10s}")
            print(f"  {'─'*9} {'─'*10} {'─'*10} {'─'*12} {'─'*10}")
            t0 = time.time()

            while True:
                elapsed = time.time() - t0
                if ramp:
                    # 斜坡插值：以恒定速度逼近目标
                    if elapsed >= ramp_time:
                        cmd_pos = target_pos
                    else:
                        frac = elapsed / ramp_time
                        cmd_pos = start_pos + frac * (target_pos - start_pos)
                else:
                    cmd_pos = target_pos

                result = motor.motion_control(position=cmd_pos, velocity=0.0,
                                              kp=args.kp, kd=args.kd)
                if result:
                    print(f"  {elapsed:6.1f}s  {cmd_pos:10.4f} {safe_fmt(result.get('position_rad'))} {safe_fmt(result.get('velocity_rad_s'))} {safe_fmt(result.get('torque_Nm'))}")
                else:
                    print(f"  {elapsed:6.1f}s  {cmd_pos:10.4f} {'无响应':>10s}")

                # 到达目标后继续保持但降低打印频率
                if not ramp or elapsed >= ramp_time:
                    time.sleep(0.2)
                else:
                    time.sleep(0.05)
        except KeyboardInterrupt:
            print(f"\n  [*] 停止")
            motor.disable()
        finally:
            motor.close()
        return

    # 诊断模式
    all_status = {}
    for t in targets:
        status = diagnose_one(t['iface'], args.master_id, args.actuator_type, t['can_eid'], t['label'])
        if status:
            all_status[t['name']] = status

    # 汇总
    if len(targets) > 1 and all_status:
        print(f"\n{'='*50}")
        print(f"  汇总")
        print(f"{'='*50}")
        print(f"  {'关节':6s} {'位置(rad)':>10s} {'速度(rad/s)':>12s} {'力矩(Nm)':>10s} {'温度(°C)':>10s} {'故障/状态':s}")
        print(f"  {'─'*6} {'─'*10} {'─'*12} {'─'*10} {'─'*10} {'─'*18}")
        for name, s in all_status.items():
            flags = []
            if s.get('uncalibrated'):        flags.append('未标定')
            if s.get('stall_overload'):      flags.append('堵转')
            if s.get('mag_fault'):           flags.append('磁编码')
            if s.get('overtemp'):            flags.append('过温')
            if s.get('phase_current_fault'): flags.append('相电流')
            if s.get('undervoltage'):        flags.append('欠压')

            # 检查故障寄存器
            faults = s.get('_faults', [])
            if faults:
                # 取最关键的故障位简短显示
                short_faults = []
                for f in faults[:2]:  # 最多显示 2 个
                    # 提取简要描述
                    if ':' in f:
                        short_faults.append(f.split(':', 1)[1])
                    else:
                        short_faults.append(f)
                flags.append('|'.join(short_faults))

            # 检查 MCU 冻结诊断
            diag = s.get('_diag', {})
            if not diag.get('heartbeat_seen'):
                flags.append('无响应')
            else:
                pos_raw = diag.get('position_raw')
                vel_raw = diag.get('velocity_raw')
                if pos_raw == 0x7FFF and vel_raw == 0x7FFF:
                    flags.append('MCU冻结')

            fault_str = ','.join(flags) if flags else '✓'
            print(f"  {name:6s} {safe_fmt(s.get('position_rad'))} {safe_fmt(s.get('velocity_rad_s'))} {safe_fmt(s.get('torque_Nm'))} {safe_fmt(s.get('temperature_C'), '>10.1f')} {fault_str}")

        # 如果检测到 MCU 冻结，打印额外提示
        frozen_motors = []
        fault_motors = []
        for name, s in all_status.items():
            diag = s.get('_diag', {})
            if '冻结' in diag.get('diagnosis', ''):
                frozen_motors.append(name)
            if s.get('_faults'):
                fault_motors.append(name)
        if frozen_motors:
            print(f"\n  ⚠ MCU 冻结电机: {', '.join(frozen_motors)}")
            print(f"     这些电机保持最后力矩输出（关节可能僵硬），需断电重启恢复。")
        if fault_motors:
            print(f"\n  ⚠ 故障寄存器非零电机: {', '.join(fault_motors)}")
            print(f"     详见上方各电机的 [3b] 故障寄存器输出。")

    print("\n[✓] 完成")


if __name__ == '__main__':
    main()
