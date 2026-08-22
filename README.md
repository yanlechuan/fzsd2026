# fzsd2026 — Reinforcement Learning Control Framework for Custom Quadruped Robot "my_dog"

[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-blue.svg?logo=ubuntu)](https://ubuntu.com/)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-brightgreen.svg?logo=ros)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic-lightgrey.svg?logo=gazebo)](http://gazebosim.org/)
[![License](https://img.shields.io/badge/license-Apache2.0-yellow.svg?logo=apache)](https://opensource.org/license/apache-2-0)

> This project is based on the [rl_sar](https://github.com/fan-ziqi/rl_sar) (v4.0.0) framework, deeply customized for the self-developed quadruped robot **my_dog**. Special thanks to the original author [Ziqi Fan](https://github.com/fan-ziqi) and all contributors for their outstanding work.

[中文文档](README_CN.md)

## Table of Contents

- [Overview](#overview)
- [Differences from the Original rl_sar](#differences-from-the-original-rl_sar)
  - [✨ Added](#-added)
  - [🔧 Streamlined](#-streamlined)
  - [🏗 Retained Core Framework](#-retained-core-framework)
  - [🔧 RobStride Motor Driver Optimization Details](#-robstride-motor-driver-optimization-details)
- [Hardware Platform](#hardware-platform)
- [Quick Start](#quick-start)
  - [System Dependencies](#system-dependencies)
  - [Build](#build)
- [Usage](#usage)
  - [Gazebo Simulation](#gazebo-simulation)
  - [Real-robot Deployment](#real-robot-deployment)
  - [Control Reference](#control-reference)
- [Pretrained Policies](#pretrained-policies)
- [Project Structure](#project-structure)
- [License](#license)
- [Acknowledgements](#acknowledgements)
- [Third-party Dependencies](#third-party-dependencies)
- [Citation](#citation)

## Overview

This project provides a complete reinforcement learning control framework for the self-developed quadruped robot **my_dog**, covering the full workflow of **simulation validation → real-world deployment**. Key features:

- **Self-developed my_dog robot**: 12-DOF quadruped robot (4 legs × 3 joints)
- **RobStride motor CAN driver**: ROS2-based multi-CAN bus motor control
- **RL policy deployment**: supports both onnxruntime and libtorch inference backends
- **Gazebo simulation**: complete my_dog URDF model and simulation environment
- **Custom action sequences**: squats, wall-climbing, bridge-crossing, etc.

## Differences from the Original rl_sar

This project is heavily streamlined and customized based on [rl_sar](https://github.com/fan-ziqi/rl_sar) (v4.0.0):

### ✨ Added


| Component | Description |
| ------------------------------------- | ------------------------------------------------------------------- |
| `src/robstride_ros2/` | **RobStride motor ROS2 driver package** — multi-thread multi-CAN architecture, see optimization details below |
| `src/robot_joint_controller/` | **Custom joint controller** — ROS2 control hardware interface adapted for my_dog |
| `src/robot_msgs/` | **Custom message package** — MotorState, RobotCommand, RobotState, IMU, etc. |
| `src/rl_sar/src/rl_real_my_dog.cpp` | **Real-robot deployment program** — my_dog-specific state machine and communication |
| `src/rl_sar/fsm_robot/fsm_my_dog.hpp` | **my_dog state machine** — action sequences such as squatting, wall-climbing, bridge-crossing |
| `policy/my_dog/` | **Pretrained policies** — multiple robot_lab versions |
| `src/rl_sar_zoo/my_dog_description/` | **my_dog robot model** — URDF/XACRO/MJCF |
| `scripts/` | **Utility scripts** — check_motor, watch_lin_vel, diagnose_controller, etc. |
| `setup.sh` | **CAN interface one-click configuration script** |

### 🔧 Streamlined

- Removed all other robot support from the original (A1, Go2, Go2W, B2, B2W, G1, GR1T1, GR1T2, L4W4, Lite3, Tita)
- Streamlined `include/` headers, keeping only what my_dog needs
- Removed ROS1 (Noetic) and ROS2 Foxy compatibility code
- Removed macOS support
- Removed Python-specific content

### 🏗 Retained Core Framework

- `inference_runtime` — inference runtime (libtorch/onnxruntime)
- `rl_sdk` — reinforcement learning control SDK
- `motion_loader` — motion sequence loader
- `observation_buffer` — observation buffer
- Gazebo / MuJoCo simulation framework
- FSM state machine framework

### 🔧 RobStride Motor Driver Optimization Details

`src/robstride_ros2/` is heavily refactored based on the [official RobStride sample](https://github.com/RobStride/robstride_ros_sample), upgrading from a **single-motor demo** to a **production-grade multi-motor cluster control system**.

#### Architecture Comparison


| Dimension | Official Sample | This Project | Improvement |
| ---------- | ---------- | ----------------- | ------- |
| Motors controlled | 1 | 12 | 🚀 12x |
| CAN buses | 1 | 4 parallel | 🚀 4x |
| Architecture | Single-thread serial | Dedicated thread per CAN bus | 🚀 |
| Configuration | Hardcoded | Dynamic ROS2 parameters | 🚀 |
| State publishing | ❌ None | 400 Hz real-time | 🚀 Added |

#### Key Optimizations

**1. Multi-thread CAN Manager (`CanDeviceThread`)**
Each CAN bus has its own control thread with a built-in command queue (driven by `condition_variable`). All 4 CAN buses run in parallel without blocking each other.

**2. recv Timeout Protection**

```cpp
// Official: recv() no timeout → packet loss causes permanent blocking → system deadlock
// This project: 10ms timeout → silent recovery after packet loss → system keeps running
struct timeval tv{};
tv.tv_usec = 10000;  // 10ms timeout
setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, ...);
```

**3. Dynamic Frequency Compensation**

```
Target period: 2000μs (500Hz)
Actual logic: processing_time < 2000 → sleep(2000 - processing_time)
             processing_time ≥ 2000 → immediately enter next cycle
```

Automatically adapts to different processor performance, keeping control frequency stable without drift.

**4. Built-in Frequency Monitoring**
Outputs the actual control frequency, processing latency, and processed motor count every second for tuning and diagnostics.

#### Low-Performance CPU Simulation Comparison (N100-class)


| Metric | Official | This Project | Note |
| ---------- | --------------------- | ---------------------- | --------------------------------- |
| Control frequency | 16.7 Hz | **416.4 Hz** 🚀 | Official is far below the RL policy requirement (50~200Hz) |
| Command throughput | 200 cmd/s | **1249 cmd/s** 🚀 | **6.3x** throughput of this project |
| Behavior on packet loss | Permanent blocking deadlock ❌ | 10ms timeout safe recovery ✅ | 55 losses with 0 freezes |
| Parallel scaling | 1 CAN × 12 motors serial | 4 CAN × 3 motors parallel | The greater the advantage on weaker CPUs |

> The lower the processor performance, the more significant the advantage of the multi-thread parallel architecture. On low-end CPUs, the official version struggles to meet real-time requirements due to serially processing many motors on a single thread, plus the deadlock risk from `recv` without timeout.

## Hardware Platform


| Component | Description |
| ---- | ------------------------------------ |
| Main controller | NVIDIA Jetson (Orin series) |
| Motors | RobStride joint motors × 12 |
| Communication | CAN bus (4 channels: can0~can3) |
| Joints | 12 DOF (4 legs × 3 joints: hip/thigh/calf) |
| IMU | hipnuc IMU |
| System | Ubuntu 22.04 + ROS2 Humble |

## Quick Start

### System Dependencies

```bash
sudo apt install cmake g++ build-essential libyaml-cpp-dev libeigen3-dev \
  libboost-all-dev libspdlog-dev libfmt-dev libtbb-dev liblcm-dev

# ROS2 related
sudo apt install ros-humble-control-toolbox ros-humble-hardware-interface \
  ros-humble-controller-interface ros-humble-controller-manager

# Gazebo
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control

# Others
sudo apt install ros-humble-teleop-twist-keyboard ros-humble-xacro
```

### Build

```bash
./build.sh
source install/setup.bash
```

> Build specific packages only: `./build.sh package1 package2`
> Clean build: `./build.sh -c`
> Build hardware-deployment code only (no ROS): `./build.sh -m`
> See `./build.sh -h` for detailed usage

## Usage

### Gazebo Simulation

```bash
ros2 launch rl_sar rl_sim.launch.py rname:=my_dog wname:=empty
```

### Real-robot Deployment

```bash
# 1. Configure CAN interfaces
sudo bash setup.sh

# 2. Start the motor driver
ros2 launch robstride_ros2 rs_motor_ros2_launch.py

# 3. Start RL control
ros2 run rl_sar rl_real_my_dog --ros-args -p rname:=my_dog
```

### Control Reference

There are three control methods: **keyboard / gamepad / RC remote controller** (RC), operated as shown in the tables below.

> **RC remote controller notes** (ET08A, Mode 2 layout): the mapping of sticks and switches is shown below; switch positions: `HIGH` = flipped to the upper limit, `LOW` = flipped to the lower limit, `MID` = middle position (3-position switches only).

#### Movement

| Function | Keyboard | Gamepad | RC |
| ---- | ---- | ---- | ---------- |
| Forward / Backward | `W` / `S` | Left stick Y (up/down) | Left stick vertical CH2 |
| Strafe left / right | `A` / `D` | Left stick X (left/right) | Left stick horizontal CH4 |
| Turn left / right (Yaw) | `Q` / `E` | Right stick X (left/right) | Right stick horizontal CH1 |
| Clear movement commands | `Space` | Release stick | Center the stick |

#### State Switching

| Function | Keyboard | Gamepad | RC |
| ---- | ---- | ---- | ---------- |
| Stand up (GetUp) | `Num0` | `A` | SC flipped to `HIGH` |
| Lie down (GetDown) | `Num9` | `B` | — |
| Calf swing (CalfSwing) | `Num5` | — | — |
| Climb wall (ClimbWall) | `Num6` | `RT + D-Pad Up` | — |
| Cross bridge (Bridge) | `Num7` | `RT + D-Pad Down` | — |
| Squat (Squat) | `Num8` | — | — |
| Toggle navigation mode | `N` | `X` | — |

#### Skill Switching

| Function | Keyboard | Gamepad | RC |
| ---- | ---- | ---- | ---------- |
| RL Locomotion 1 (basic walking) | `Num1` | `RB + D-Pad Up` | SA flipped to `HIGH` |
| RL Locomotion 2 | `Num2` | `RB + D-Pad Down` | SA flipped to `LOW` |
| RL Locomotion 3 | `Num3` | `RB + D-Pad Left` | — |
| RL Locomotion 4 | `Num4` | `RB + D-Pad Right` | — |

#### Motor Control

| Function | Keyboard | Gamepad | RC |
| ---- | ---- | ---- | ---------- |
| Motor enable (fix.sh) | — | `LB + A` | SB flipped to `HIGH` |
| Motor power-off (robstride_off.sh) | — | `LB + B` | SB flipped to `LOW` |
| Passive mode (kp=0, kd=8) | `P` | `LB + X` | SC flipped to `LOW` (E-stop) |
| Reset robot in simulation | `R` | — | — |
| Power off | — | `LT + B` | — |

> **RD knob (CH8)**: the continuous knob of the RC remote controller, used for online adjustment of continuous parameters such as height/speed (mapped to `knobs`).

## Pretrained Policies

Multiple versions of policy files are located in `policy/my_dog/` (robot_lab, robot_lab2, robot_lab3, robot_lab4). To train policies, use the [robot_lab](https://github.com/fan-ziqi/robot_lab) project, then export the ONNX model and place it in the corresponding directory.

## Project Structure

```
fzsd2026/
├── src/
│   ├── rl_sar/                    # Core framework (streamlined)
│   ├── rl_sar_zoo/                # my_dog robot model
│   ├── robstride_ros2/            # ★ RobStride motor driver
│   ├── robot_joint_controller/    # ★ Joint controller
│   ├── robot_msgs/                # ★ Custom messages
│   ├── hipnuc_imu/                # ★ IMU driver
│   └── hipnuc_lib_package/        # ★ IMU algorithm library
├── policy/my_dog/                 # Pretrained policies
├── scripts/                       # Utility scripts
├── build.sh                       # Build script
└── setup.sh                       # CAN configuration script
```

## License

This project is based on [rl_sar](https://github.com/fan-ziqi/rl_sar) (v4.0.0) and uses its [Apache License 2.0](LICENSE).

## Acknowledgements

This project references and is adapted from the following excellent open-source projects, with special thanks:

- [rl_sar](https://github.com/fan-ziqi/rl_sar) — provides the core SAR framework
- [robot_lab](https://github.com/fan-ziqi/robot_lab) — IsaacLab training framework
- [robstride_ros_sample](https://github.com/RobStride/robstride_ros_sample) — official RobStride motor sample
- Original author [Ziqi Fan](https://github.com/fan-ziqi) and all contributors

## Third-party Dependencies

This project contains several third-party libraries under `src/rl_sar/library/thirdparty/`, distributed along with the upstream framework, each following its own open-source license:

| Third-party library | Path | License |
| -------- | ---- | ------ |
| [joystick](https://www.kernel.org/doc/html/latest/input/joydev.html) | `library/thirdparty/joystick/` | Apache-2.0 (`LICENSE-2.0.txt`) |
| [MuJoCo](https://github.com/google-deepmind/mujoco) simulation UI | `library/thirdparty/mujoco_simulate/` | Apache-2.0 |
| [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) | `library/thirdparty/robot_sdk/unitree/unitree_sdk2/` | see its `LICENSE` |
| [unitree_legged_sdk](https://github.com/unitreerobotics/unitree_legged_sdk) | `library/thirdparty/robot_sdk/unitree/unitree_legged_sdk/` | see its `LICENSE` |
| Lite3 Motion Control SDK | `library/thirdparty/robot_sdk/deeprobotics/Lite3_MotionSDK/` | see its `LICENSE` |
| [Deeprobotics gamepad](https://github.com/DeepRoboticsLab/gamepad) | `library/thirdparty/robot_sdk/deeprobotics/gamepad/` | see its `LICENSE` |

> These third-party libraries are introduced by the upstream [rl_sar](https://github.com/fan-ziqi/rl_sar) framework. If you need to use or redistribute any part of them, please comply with the corresponding library's own open-source license terms, and refer to the actual LICENSE file carried by each library.

## Citation

If you use or reference this project's code, please cite both this project and the upstream [rl_sar](https://github.com/fan-ziqi/rl_sar):

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

> **Disclaimer: The user acknowledges that all risks and consequences arising from using this code shall be solely borne by the user, and the author assumes no direct or indirect liability. Adequate safety protection measures must be ensured before operation.**
