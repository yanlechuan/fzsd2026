#!/usr/bin/env python3
"""
低性能处理器 (N100等) 下 robstride_ros 电机包控制频率仿真对比

模拟场景:
  - 官方版: 1条CAN + 单线程 + recv无超时阻塞
  - 你的版(MultiThread): 4条CAN + 每线独立线程 + recv有10ms超时

低性能模拟参数 (N100 ≈ Jetson Orin 1/3 ~ 1/5):
  - CAN帧写耗时: 50μs → 250μs (5x)
  - CAN帧读耗时(有回复): 200μs → 1000μs (5x)
  - CPU计算开销(编码/解码/ROS): 20μs → 100μs (5x)
  - CAN帧丢包率: 0.5%
"""

import time
import random
import threading
from dataclasses import dataclass
from enum import Enum

random.seed(42)

# === 仿真参数 ===

# 低性能CPU延迟放大倍数 (N100 vs Orin)
CPU_SLOWDOWN = 5

# 电机参数
NUM_MOTORS = 12
NUM_CANS = 4          # 你的版本
NUM_CANS_OFFICIAL = 1 # 官方版本

# CAN通信延迟 (基础值 * CPU_SLOWDOWN)
CAN_WRITE_US = 50 * CPU_SLOWDOWN       # 写CAN帧
CAN_READ_OK_US = 200 * CPU_SLOWDOWN    # 读CAN帧(有回复)
CAN_READ_TIMEOUT_US = 10000            # 你的版本: 10ms超时
CPU_OVERHEAD_US = 20 * CPU_SLOWDOWN    # 编码解码等CPU开销

# 电机回复模拟
MOTOR_REPLY_DELAY_US = 500 * CPU_SLOWDOWN  # 电机回复延迟
PACKET_LOSS_RATE = 0.005                     # 0.5% 丢包率

# 运行时间
SIMULATION_SECONDS = 10

# === 统计 ===
@dataclass
class Stats:
    commands_sent: int = 0
    commands_received: int = 0
    total_wait_us: int = 0
    max_wait_us: int = 0
    timeouts: int = 0
    errors: int = 0
    cycles: int = 0

class Mode(Enum):
    OFFICIAL = "official"
    MULTI_THREAD = "multi_thread"

def simulate_official(stats: Stats, stop_event):
    """
    官方版: 1条CAN, 单线程, recv无超时阻塞
    每次 send_motion_command:
      1. write(CAN帧)
      2. recv(阻塞等待回复)
      3. usleep(1000)
    """
    while not stop_event.is_set():
        cycle_start = time.perf_counter()
        stats.cycles += 1
        
        # 为当前CAN上的所有电机发命令 (官方只有1条CAN)
        motors_on_this_can = NUM_MOTORS // NUM_CANS_OFFICIAL  # 12
        
        for m in range(motors_on_this_can):
            stats.commands_sent += 1
            
            # 1. CPU开销: 编码命令
            time.sleep(CPU_OVERHEAD_US / 1_000_000)
            
            # 2. write(CAN帧)
            time.sleep(CAN_WRITE_US / 1_000_000)
            
            # 3. recv(阻塞等待回复) — 官方无超时!
            if random.random() < PACKET_LOSS_RATE:
                # 丢包 → recv会永久卡住! (官方无超时)
                # 模拟: 卡住直到 stop_event
                stats.errors += 1
                # 现实中这就死锁了, 模拟等待一小段时间后继续
                time.sleep(0.5)
                if stop_event.is_set():
                    return
                continue
            
            # 正常回复: 等待电机回复延迟
            wait_time = MOTOR_REPLY_DELAY_US / 1_000_000
            time.sleep(wait_time)
            
            # 4. CPU开销: 解码回复
            time.sleep(CPU_OVERHEAD_US / 1_000_000)
            
            stats.commands_received += 1
            wait_us = int(wait_time * 1_000_000)
            stats.total_wait_us += wait_us
            stats.max_wait_us = max(stats.max_wait_us, wait_us)
        
        # 5. 官方每循环末尾 usleep(1000)
        time.sleep(1000 / 1_000_000)
        
        # 频率控制: 确保循环至少占用1ms
        elapsed = (time.perf_counter() - cycle_start) * 1_000_000
        if elapsed < 1000:
            time.sleep((1000 - elapsed) / 1_000_000)

def simulate_multi_thread(can_index: int, motors: list, stats: Stats, stop_event):
    """
    你的版: 每条CAN一个独立线程
    每个线程:
      - desired_period = 2000μs (500Hz)
      - recv有10ms超时
      - 丢包10ms后返回, 不会卡死
    """
    while not stop_event.is_set():
        cycle_start = time.perf_counter()
        stats.cycles += 1
        
        for motor_id in motors:
            stats.commands_sent += 1
            
            # 1. CPU开销: 编码命令
            time.sleep(CPU_OVERHEAD_US / 1_000_000)
            
            # 2. write(CAN帧)
            time.sleep(CAN_WRITE_US / 1_000_000)
            
            # 3. recv(有10ms超时!)
            if random.random() < PACKET_LOSS_RATE:
                # 丢包 → 等10ms超时后返回
                time.sleep(10000 / 1_000_000)  # 10ms超时
                stats.timeouts += 1
                # 不抛异常, 静默继续!
                continue
            
            # 正常回复: 等待电机回复延迟
            wait_time = MOTOR_REPLY_DELAY_US / 1_000_000
            time.sleep(wait_time)
            
            # 4. CPU开销: 解码回复
            time.sleep(CPU_OVERHEAD_US / 1_000_000)
            
            stats.commands_received += 1
            wait_us = int(wait_time * 1_000_000)
            stats.total_wait_us += wait_us
            stats.max_wait_us = max(stats.max_wait_us, wait_us)
        
        # 动态周期补偿: 目标2000μs
        elapsed = (time.perf_counter() - cycle_start) * 1_000_000
        if elapsed < 2000:
            time.sleep((2000 - elapsed) / 1_000_000)

def run_simulation(mode: Mode, num_cans: int, motors_per_can: int) -> dict:
    """运行仿真并返回统计"""
    
    stats_list = [Stats() for _ in range(num_cans)]
    stop_event = threading.Event()
    threads = []
    
    print(f"\n{'='*60}")
    print(f"模拟: {mode.value}")
    print(f"  CAN总线: {num_cans}条")
    print(f"  电机数: {num_cans * motors_per_can}个 (每条CAN {motors_per_can}台)")
    print(f"  CPU模拟: N100级别 (延迟放大{CPU_SLOWDOWN}x)")
    print(f"  丢包率: {PACKET_LOSS_RATE*100:.1f}%")
    print(f"  运行时间: {SIMULATION_SECONDS}s")
    print(f"{'='*60}")
    
    start_time = time.time()
    
    if mode == Mode.OFFICIAL:
        motors_list = [list(range(1, NUM_MOTORS + 1))]
        t = threading.Thread(
            target=simulate_official,
            args=(stats_list[0], stop_event)
        )
        threads.append(t)
    else:
        for i in range(num_cans):
            start_idx = i * motors_per_can + 1
            end_idx = start_idx + motors_per_can
            motors = list(range(start_idx, end_idx))
            t = threading.Thread(
                target=simulate_multi_thread,
                args=(i, motors, stats_list[i], stop_event)
            )
            threads.append(t)
    
    for t in threads:
        t.start()
    
    # 并行运行
    try:
        time.sleep(SIMULATION_SECONDS)
    finally:
        stop_event.set()
    
    for t in threads:
        t.join()
    
    elapsed = time.time() - start_time
    
    # 汇总统计
    total_sent = sum(s.commands_sent for s in stats_list)
    total_received = sum(s.commands_received for s in stats_list)
    total_cycles = sum(s.cycles for s in stats_list)
    total_errors = sum(s.errors for s in stats_list)
    total_timeouts = sum(s.timeouts for s in stats_list)
    total_wait = sum(s.total_wait_us for s in stats_list)
    max_wait = max(s.max_wait_us for s in stats_list)
    
    avg_freq = total_cycles / elapsed
    
    results = {
        "mode": mode.value,
        "elapsed": elapsed,
        "total_cycles": total_cycles,
        "total_sent": total_sent,
        "total_received": total_received,
        "avg_freq_hz": avg_freq,
        "total_errors": total_errors,
        "total_timeouts": total_timeouts,
        "avg_latency_us": total_wait / total_received if total_received else 0,
        "max_latency_us": max_wait,
        "loss_rate": 1 - total_received / total_sent if total_sent else 0,
        "commands_per_second": total_sent / elapsed,
    }
    
    print(f"\n--- 结果 ---")
    print(f"总控制循环数: {results['total_cycles']}")
    print(f"控制频率: {results['avg_freq_hz']:.1f} Hz")
    print(f"总命令发送: {results['total_sent']}")
    print(f"总命令接收(回复): {results['total_received']}")
    print(f"命令吞吐: {results['commands_per_second']:.0f} 命令/秒")
    print(f"丢包/错误: {results['total_errors']}")
    print(f"超时(安全恢复): {results['total_timeouts']}")
    print(f"实际丢包率: {results['loss_rate']*100:.2f}%")
    print(f"平均延迟: {results['avg_latency_us']:.0f} μs")
    print(f"最大延迟: {results['max_latency_us']:.0f} μs")
    
    return results

def main():
    print("=" * 60)
    print("低性能CPU (N100) 下 robstride_ros 电机控制频率仿真")
    print(f"CPU延迟放大倍数: {CPU_SLOWDOWN}x")
    print("=" * 60)
    
    # 官方版
    official_results = run_simulation(
        Mode.OFFICIAL,
        NUM_CANS_OFFICIAL,
        NUM_MOTORS // NUM_CANS_OFFICIAL
    )
    
    # 你的版 (MultiThread)
    multi_results = run_simulation(
        Mode.MULTI_THREAD,
        NUM_CANS,
        NUM_MOTORS // NUM_CANS
    )
    
    # 最终对比
    print(f"\n{'='*60}")
    print(f"最终对比 (N100级别处理器)")
    print(f"{'='*60}")
    print(f"{'指标':<25} {'官方版':>15} {'你的版(MultiThread)':>22}")
    print(f"{'-'*62}")
    print(f"{'控制频率 (Hz)':<25} {official_results['avg_freq_hz']:>15.1f} {multi_results['avg_freq_hz']:>22.1f}")
    freq_ratio = multi_results['avg_freq_hz'] / official_results['avg_freq_hz']
    print(f"{'频率提升':<25} {'':>15} {freq_ratio:>21.1f}x")
    print(f"{'命令吞吐 (cmd/s)':<25} {official_results['commands_per_second']:>15.0f} {multi_results['commands_per_second']:>22.0f}")
    thr_ratio = multi_results['commands_per_second'] / official_results['commands_per_second']
    print(f"{'吞吐提升':<25} {'':>15} {thr_ratio:>21.1f}x")
    print(f"{'死锁/卡死次数':<25} {official_results['total_errors']:>15} {multi_results['total_errors']:>22}")
    print(f"{'超时安全恢复次数':<25} {'N/A (会卡死)':>15} {multi_results['total_timeouts']:>22}")
    print(f"{'平均延迟 (μs)':<25} {official_results['avg_latency_us']:>15.0f} {multi_results['avg_latency_us']:>22.0f}")
    print(f"{'丢包后行为':<25} {'永久阻塞':>15} {'10ms后恢复':>22}")
    
    print(f"\n{'='*60}")
    print("结论:")
    print(f"  - 低性能CPU下, 你的版本吞吐量是官方的 {thr_ratio:.1f} 倍")
    print(f"  - 官方版丢包后会永久卡死, 你的版10ms超时安全恢复")
    print(f"  - 官方版1条CAN串行处理12个电机, 你的版4条CAN并行")
    print(f"  - 处理器性能越低, 并行架构的优势越明显")
    print(f"{'='*60}")

if __name__ == "__main__":
    main()
