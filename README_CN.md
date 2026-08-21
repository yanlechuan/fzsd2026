# fzsd2026 — 自研四足机器人 my_dog 强化学习控制框架

[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-blue.svg?logo=ubuntu)](https://ubuntu.com/)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-brightgreen.svg?logo=ros)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic-lightgrey.svg?logo=gazebo)](http://gazebosim.org/)
[![License](https://img.shields.io/badge/license-Apache2.0-yellow.svg?logo=apache)](https://opensource.org/license/apache-2-0)

> 本项目基于 [rl_sar](https://github.com/fan-ziqi/rl_sar) (v4.0.0) 框架，针对自研四足机器人 **my_dog** 进行了深度定制。感谢原作者 [Ziqi Fan](https://github.com/fan-ziqi) 及其贡献者的出色工作。

[English](README.md)

## 项目简介

本项目为自研四足机器人 **my_dog** 提供了完整的强化学习控制框架，涵盖 **仿真验证 → 实机部署** 的全流程。主要特性：

- **自研 my_dog 机器人**：12 自由度四足机器人（4腿 × 3关节）
- **RobStride 电机 CAN 驱动**：基于 ROS2 的多路 CAN 总线电机控制
- **RL 策略部署**：支持 onnxruntime 与 libtorch 双推理后端
- **Gazebo 仿真**：完整的 my_dog URDF 模型与仿真环境
- **自定义动作**：蹲起、爬墙、过断桥等动作序列（）

## 与原版 rl_sar 的差异

本项目基于 [rl_sar](https://github.com/fan-ziqi/rl_sar) (v4.0.0) 进行了大量精简和定制：

### ✨ 新增


| 项目                                  | 说明                                                                |
| ------------------------------------- | ------------------------------------------------------------------- |
| `src/robstride_ros2/`                 | **RobStride 电机 ROS2 驱动包** — 多线程多CAN架构，详见下方优化说明 |
| `src/robot_joint_controller/`         | **自定义关节控制器** — 适配 my_dog 的 ROS2 control 硬件接口        |
| `src/robot_msgs/`                     | **自定义消息包** — MotorState, RobotCommand, RobotState, IMU 等    |
| `src/rl_sar/src/rl_real_my_dog.cpp`   | **实机部署程序** — my_dog 专用状态机与通信                         |
| `src/rl_sar/fsm_robot/fsm_my_dog.hpp` | **my_dog 状态机** — 蹲起/爬墙/过断桥等动作序列                     |
| `policy/my_dog/`                      | **预训练策略** — robot_lab 多个版本                                |
| `src/rl_sar_zoo/my_dog_description/`  | **my_dog 机器人模型** — URDF/XACRO/MJCF                            |
| `scripts/`                            | **辅助脚本** — check_motor, watch_lin_vel, diagnose_controller 等  |
| `setup.sh`                            | **CAN 接口一键配置脚本**                                            |

### 🔧 精简

- 移除了原版所有其他机器人支持（A1, Go2, Go2W, B2, B2W, G1, GR1T1, GR1T2, L4W4, Lite3, Tita）
- 精简了 `include/` 头文件，仅保留 my_dog 所需
- 移除了 ROS1 (Noetic) 和 ROS2 Foxy 的兼容代码
- 移除了 macOS 支持相关内容
- 移除了 Python 版本相关内容

### 🏗 保留的核心框架

- `inference_runtime` — 推理运行时 (libtorch/onnxruntime)
- `rl_sdk` — 强化学习控制 SDK
- `motion_loader` — 动作加载器
- `observation_buffer` — 观测值缓冲区
- Gazebo / MuJoCo 仿真框架
- FSM 状态机框架

### 🔧 RobStride 电机驱动优化详解

`src/robstride_ros2/` 基于 [RobStride 官方样例](https://github.com/RobStride/robstride_ros_sample) 进行了深度重构，从**单电机演示**升级为**生产级多电机集群控制系统**。

#### 架构对比


| 对比维度   | 官方样例   | 本项目            | 提升    |
| ---------- | ---------- | ----------------- | ------- |
| 控制电机数 | 1 个       | 12 个             | 🚀 12x  |
| CAN 总线   | 1 条       | 4 条并联          | 🚀 4x   |
| 架构       | 单线程串行 | 每条 CAN 独立线程 | 🚀      |
| 参数化     | 硬编码     | ROS2 参数动态配置 | 🚀      |
| 状态发布   | ❌ 无      | 400 Hz 实时发布   | 🚀 新增 |

#### 关键优化点

**1. 多线程 CAN 管理器 (`CanDeviceThread`)**
每条 CAN 总线拥有独立的控制线程，内置命令队列（`condition_variable` 驱动），4 条 CAN 同时运行互不阻塞。

**2. recv 超时保护**

```cpp
// 官方: recv() 无超时 → 丢包则永久阻塞 → 系统死锁
// 本项目: 10ms 超时 → 丢包后静默返回 → 系统继续运行
struct timeval tv{};
tv.tv_usec = 10000;  // 10ms 超时
setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, ...);
```

**3. 动态频率补偿**

```
目标周期: 2000μs (500Hz)
实际逻辑: processing_time < 2000 → sleep(2000 - processing_time)
          processing_time ≥ 2000 → 立即进入下一周期
```

自动适应不同处理器性能，控制频率稳定不漂移。

**4. 内置频率监控**
每秒输出实际控制频率、处理延迟、已处理电机数，方便调优和诊断。

#### 低性能 CPU 仿真对比 (模拟 N100 级别)


| 指标       | 官方版                | 本项目                 | 说明                              |
| ---------- | --------------------- | ---------------------- | --------------------------------- |
| 控制频率   | 16.7 Hz               | **416.4 Hz** 🚀        | 官方远低于 RL 策略需求 (50~200Hz) |
| 命令吞吐   | 200 cmd/s             | **1249 cmd/s** 🚀      | 本项目吞吐量**6.3 倍**            |
| 丢包后行为 | 永久阻塞死锁 ✅       | 10ms 超时安全恢复 ✅   | 本项目 55 次丢包 0 次卡死         |
| 平行扩展   | 1 条 CAN 串行 12 电机 | 4 条 CAN × 3 电机并行 | 处理器越弱优势越明显              |

> 处理器性能越低，多线程并行架构的优势越显著。官方版在低端 CPU 上因单线程串行处理大量电机，加上 `recv` 无超时的死锁风险，几乎无法满足实时性要求。

## 硬件平台


| 组件 | 说明                                 |
| ---- | ------------------------------------ |
| 主控 | NVIDIA Jetson (Orin 系列)            |
| 电机 | RobStride 关节电机 × 12             |
| 通信 | CAN 总线（4路 can0~can3）            |
| 关节 | 12 DOF（4腿 × 3关节：髋/大腿/小腿） |
| IMU  | hipnuc IMU                           |
| 系统 | Ubuntu 22.04 + ROS2 Humble           |

## 快速开始

### 系统依赖

```bash
sudo apt install cmake g++ build-essential libyaml-cpp-dev libeigen3-dev \
  libboost-all-dev libspdlog-dev libfmt-dev libtbb-dev liblcm-dev

# ROS2 相关
sudo apt install ros-humble-control-toolbox ros-humble-hardware-interface \
  ros-humble-controller-interface ros-humble-controller-manager

# Gazebo
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control

# 其他
sudo apt install ros-humble-teleop-twist-keyboard ros-humble-xacro
```

### 编译

```bash
./build.sh
source install/setup.bash
```

> 仅编译特定包：`./build.sh package1 package2`
> 清理构建：`./build.sh -c`
> 仅编译实机代码（禁用 ROS）：`./build.sh -m`
> 详细用法见 `./build.sh -h`

## 使用

### Gazebo 仿真

```bash
ros2 launch rl_sar rl_sim.launch.py rname:=my_dog wname:=empty
```

### 实机部署

```bash
# 1. 配置 CAN 接口
sudo bash setup.sh

# 2. 启动电机驱动
ros2 launch robstride_ros2 rs_motor_ros2_launch.py

# 3. 启动 RL 控制
ros2 run rl_sar rl_real_my_dog --ros-args -p rname:=my_dog
```

### 控制方式

控制方式分为 **键盘 / 手柄 / 航模遥控器**（RC）三种，按下表操作。

> **航模遥控器说明**（ET08A，Mode 2 布局）：摇杆与开关的映射关系见下表；其中开关档位：`HIGH`=拨到上限、`LOW`=拨到下限、`MID`=中间档（仅 3 档开关）。

#### 移动控制

| 功能 | 键盘 | 手柄 | 航模（RC） |
| ---- | ---- | ---- | ---------- |
| 前进 / 后退 | `W` / `S` | 左摇杆 Y（上下） | 左摇杆纵 CH2 |
| 左移 / 右移 | `A` / `D` | 左摇杆 X（左右） | 左摇杆横 CH4 |
| 左转 / 右转（Yaw） | `Q` / `E` | 右摇杆 X（左右） | 右摇杆横 CH1 |
| 清零移动指令 | `Space` | 松开摇杆 | 摇杆回中 |

#### 状态切换

| 功能 | 键盘 | 手柄 | 航模（RC） |
| ---- | ---- | ---- | ---------- |
| 站立（GetUp） | `Num0` | `A` | SC 拨到 `HIGH` |
| 趴下（GetDown） | `Num9` | `B` | — |
| 小腿摆动（CalfSwing） | `Num5` | — | — |
| 爬墙（ClimbWall） | `Num6` | `RT + D-Pad 上` | — |
| 过断桥（Bridge） | `Num7` | `RT + D-Pad 下` | — |
| 蹲起（Squat） | `Num8` | — | — |
| 切换导航模式 | `N` | `X` | — |

#### 技能切换

| 功能 | 键盘 | 手柄 | 航模（RC） |
| ---- | ---- | ---- | ---------- |
| RL Locomotion 1（基础行走） | `Num1` | `RB + D-Pad 上` | SA 拨到 `HIGH` |
| RL Locomotion 2 | `Num2` | `RB + D-Pad 下` | SA 拨到 `LOW` |
| RL Locomotion 3 | `Num3` | `RB + D-Pad 左` | — |
| RL Locomotion 4 | `Num4` | `RB + D-Pad 右` | — |

#### 电机控制

| 功能 | 键盘 | 手柄 | 航模（RC） |
| ---- | ---- | ---- | ---------- |
| 电机使能（fix.sh） | — | `LB + A` | SB 拨到 `HIGH` |
| 电机断电（robstride_off.sh） | — | `LB + B` | SB 拨到 `LOW` |
| 被动模式（kp=0, kd=8） | `P` | `LB + X` | SC 拨到 `LOW`（急停） |
| 仿真中重置机器人 | `R` | — | — |
| 关机 | — | `LT + B` | — |

> **RD 旋钮（CH8）**：航模遥控器的连续旋钮，用于高度/速度等连续参数的在线调节（映射到 `knobs`）。

## 预训练策略

多个版本的策略文件位于 `policy/my_dog/`（robot_lab、robot_lab2、robot_lab3、robot_lab4）。如需训练策略，请使用 [robot_lab](https://github.com/fan-ziqi/robot_lab) 项目，训练后导出 ONNX 模型放入对应目录即可。

## 项目目录

```
fzsd2026/
├── src/
│   ├── rl_sar/                    # 核心框架（精简版）
│   ├── rl_sar_zoo/                # my_dog 机器人模型
│   ├── robstride_ros2/            # ★ RobStride 电机驱动
│   ├── robot_joint_controller/    # ★ 关节控制器
│   ├── robot_msgs/                # ★ 自定义消息
│   ├── hipnuc_imu/                # ★ IMU 驱动
│   └── hipnuc_lib_package/        # ★ IMU 算法库
├── policy/my_dog/                 # 预训练策略
├── scripts/                       # 工具脚本
├── build.sh                       # 构建脚本
└── setup.sh                       # CAN 配置脚本
```

## 许可证

本项目基于 [rl_sar](https://github.com/fan-ziqi/rl_sar) (v4.0.0)，沿用其 [Apache License 2.0](LICENSE)。

## 致谢

本项目参考并改编自以下优秀开源项目，特此致谢：

- [rl_sar](https://github.com/fan-ziqi/rl_sar) — 提供核心 SAR 框架
- [robot_lab](https://github.com/fan-ziqi/robot_lab) — IsaacLab 训练框架
- [robstride_ros_sample](https://github.com/RobStride/robstride_ros_sample) — RobStride 电机官方样例
- 原作者 [Ziqi Fan](https://github.com/fan-ziqi) 及其所有贡献者

## 第三方依赖

本项目包含 `src/rl_sar/library/thirdparty/` 目录下的若干第三方库，随上游框架一并分发，均遵循其各自的开源许可证，特此说明：

| 第三方库 | 路径 | 许可证 |
| -------- | ---- | ------ |
| [joystick](https://www.kernel.org/doc/html/latest/input/joydev.html) | `library/thirdparty/joystick/` | Apache-2.0 (`LICENSE-2.0.txt`) |
| [MuJoCo](https://github.com/google-deepmind/mujoco) 仿真 UI | `library/thirdparty/mujoco_simulate/` | Apache-2.0 |
| [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) | `library/thirdparty/robot_sdk/unitree/unitree_sdk2/` | 见其 `LICENSE` |
| [unitree_legged_sdk](https://github.com/unitreerobotics/unitree_legged_sdk) | `library/thirdparty/robot_sdk/unitree/unitree_legged_sdk/` | 见其 `LICENSE` |
| Lite3 运动控制 SDK | `library/thirdparty/robot_sdk/deeprobotics/Lite3_MotionSDK/` | 见其 `LICENSE` |
| [Deeprobotics gamepad](https://github.com/DeepRoboticsLab/gamepad) | `library/thirdparty/robot_sdk/deeprobotics/gamepad/` | 见其 `LICENSE` |

> 这些第三方库由上游 [rl_sar](https://github.com/fan-ziqi/rl_sar) 框架引入。若您需要使用或再分发其中的某一部分，请务必遵守对应库自身的开源许可条款，并以各库实际携带的 LICENSE 文件为准。

## 引用

如果您使用或参考了本项目代码，请同时引用本项目与上游 [rl_sar](https://github.com/fan-ziqi/rl_sar)：

```bibtex
@software{fan-ziqi2024rl_sar,
  author = {fan-ziqi},
  title = {rl_sar: Simulation Verification and Physical Deployment of Robot Reinforcement Learning Algorithm.},
  url = {https://github.com/fan-ziqi/rl_sar},
  year = {2024}
}

@software{fzsd2026,
  author = {yanlechuan},
  title = {fzsd2026: Self-developed Quadruped Robot my_dog RL Control Framework},
  url = {https://github.com/yanlechuan/fzsd2026},
  year = {2026}
}
```

---

> **免责声明：使用者确认使用本代码产生的所有风险及后果均由使用者自行承担，作者不承担任何直接或间接责任，操作前必须确保已采取充分安全防护措施。**
