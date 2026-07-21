# fzsd2026 — Reinforcement Learning Control Framework for Custom Quadruped Robot "my_dog"

[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-blue.svg?logo=ubuntu)](https://ubuntu.com/)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-brightgreen.svg?logo=ros)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic-lightgrey.svg?logo=gazebo)](http://gazebosim.org/)
[![License](https://img.shields.io/badge/license-Apache2.0-yellow.svg?logo=apache)](https://opensource.org/license/apache-2-0)

[中文文档](README_CN.md)

> This project is based on [rl_sar](https://github.com/fan-ziqi/rl_sar) (v4.0.0), customized for the self-developed quadruped robot **my_dog**. Special thanks to the original author [Ziqi Fan](https://github.com/fan-ziqi) and all contributors.

## Overview

This project provides a complete reinforcement learning control framework for the custom quadruped robot **my_dog**, covering **simulation → real-world deployment** workflow. Key features:

- **my_dog Robot**: 12-DOF quadruped (4 legs × 3 joints: hip/thigh/calf)
- **RobStride CAN Motor Driver**: ROS2-based multi-CAN bus motor control
- **RL Policy Deployment**: Supports both onnxruntime and libtorch inference backends
- **Gazebo Simulation**: Complete URDF model and simulation environment
- **Custom Action Sequences**: Squat, wall-climb, bridge-crossing, etc.

## Differences from rl_sar

This project is derived from [rl_sar](https://github.com/fan-ziqi/rl_sar) (v4.0.0) with the following changes:

### ✨ Added
| Component | Description |
|-----------|-------------|
| `src/robstride_ros2/` | **RobStride motor ROS2 driver** — multi-thread multi-CAN architecture, see optimization details below |
| `src/robot_joint_controller/` | **Custom joint controller** — ROS2 control hardware interface for my_dog |
| `src/robot_msgs/` | **Custom ROS2 messages** — MotorState, RobotCommand, RobotState, IMU |
| `src/rl_sar/src/rl_real_my_dog.cpp` | **Real robot deployment** — my_dog specific FSM and communication |
| `src/rl_sar/fsm_robot/fsm_my_dog.hpp` | **my_dog state machine** — squat/wall-climb/bridge-crossing sequences |
| `policy/my_dog/` | **Pretrained policies** — multiple robot_lab versions |
| `src/rl_sar_zoo/my_dog_description/` | **my_dog robot model** — URDF/XACRO/MJCF |
| `scripts/` | **Utility scripts** — motor check, velocity monitor, controller diagnostics |
| `setup.sh` | **CAN interface one-click configuration** |

### 🔧 RobStride Motor Driver Optimization Details

The `src/robstride_ros2/` package is heavily refactored from the [official RobStride sample](https://github.com/RobStride/robstride_ros_sample), evolved from a **single-motor demo** to a **production-grade multi-motor cluster control system**.

#### Architecture Comparison

| Dimension | Official Sample | This Project | Improvement |
|-----------|----------------|--------------|-------------|
| Motors controlled | 1 motor | 12 motors | 🚀 12x |
| CAN buses | 1 bus | 4 parallel buses | 🚀 4x |
| Architecture | Single-thread serial | Dedicated thread per CAN bus | 🚀 |
| Configuration | Hardcoded | ROS2 parameters dynamic | 🚀 |
| State publishing | ❌ None | 400 Hz real-time | 🚀 New |

#### Key Optimizations

**1. Multi-thread CAN Manager (`CanDeviceThread`)**
Each CAN bus has its own control thread with a command queue driven by `condition_variable`. 4 CAN buses run in parallel without blocking each other.

**2. recv Timeout Protection**
```cpp
// Official: recv() no timeout → packet loss causes permanent deadlock
// This project: 10ms timeout → silent recovery after packet loss
struct timeval tv{};
tv.tv_usec = 10000;  // 10ms timeout
setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, ...);
```

**3. Dynamic Frequency Compensation**
```
Target period: 2000μs (500Hz)
Logic: processing_time < 2000 → sleep(2000 - processing_time)
       processing_time ≥ 2000 → immediate next cycle
```
Auto-adapts to different CPU performance levels, maintains stable control frequency.

**4. Built-in Frequency Monitor**
Logs actual control frequency, processing latency, and processed motor count every second for diagnostics.

#### Low-Performance CPU Simulation (N100-class)

| Metric | Official | This Project | Note |
|--------|----------|--------------|------|
| Control frequency | 16.7 Hz | **416.4 Hz** 🚀 | Official far below RL requirement (50~200Hz) |
| Command throughput | 200 cmd/s | **1249 cmd/s** 🚀 | **6.3x** throughput improvement |
| Behavior on packet loss | Permanent deadlock ❌ | 10ms safe recovery ✅ | 55 losses, 0 freezes |
| Scaling | 1 CAN × 12 motors serial | 4 CAN × 3 motors parallel | Greater advantage on weaker CPUs |

> The lower the CPU performance, the more significant the advantage of the multi-thread parallel architecture. The official version struggles to meet real-time requirements on low-end CPUs due to serial processing of many motors on a single thread and the risk of deadlock from `recv` without timeout.

### 🔧 Removed
- All other robot support (A1, Go2, Go2W, B2, B2W, G1, GR1T1, GR1T2, L4W4, Lite3, Tita)
- ROS1 (Noetic) and ROS2 Foxy compatibility code
- macOS support
- Python version (use [v2.3](https://github.com/fan-ziqi/rl_sar/releases/tag/v2.3) if needed)

### 🏗 Retained Core
- `inference_runtime` — inference runtime (libtorch/onnxruntime)
- `rl_sdk` — RL control SDK
- `motion_loader` — motion sequence loader
- `observation_buffer` — observation buffer
- Gazebo / MuJoCo simulation framework
- FSM state machine framework

## Hardware Platform

| Component | Description |
|-----------|-------------|
| Main Board | NVIDIA Jetson (Orin series) |
| Motors | RobStride joint motors × 12 |
| Communication | CAN bus (4 channels: can0~can3) |
| Joints | 12 DOF (4 legs × 3 joints) |
| IMU | hipnuc IMU |
| OS | Ubuntu 22.04 + ROS2 Humble |

## Quick Start

### System Dependencies

```bash
sudo apt install cmake g++ build-essential libyaml-cpp-dev libeigen3-dev \
  libboost-all-dev libspdlog-dev libfmt-dev libtbb-dev liblcm-dev

# ROS2
sudo apt install ros-humble-control-toolbox ros-humble-hardware-interface \
  ros-humble-controller-interface ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control ros-humble-teleop-twist-keyboard \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-xacro
```

### Download Third-party Libraries

```bash
bash scripts/download_inference_runtime.sh
bash scripts/download_mujoco.sh
bash scripts/download_robot_descriptions.sh
```

### Build

```bash
./build.sh
source install/setup.bash
```

> Build specific packages: `./build.sh package1 package2`
>
> Clean build: `./build.sh -c`
>
> Build for hardware deployment only (no ROS): `./build.sh -m`

## Usage

### Gazebo Simulation

```bash
ros2 launch rl_sar rl_sim.launch.py rname:=my_dog wname:=empty
```

### Real Robot Deployment

```bash
# 1. Configure CAN interfaces
sudo bash setup.sh

# 2. Start motor driver
ros2 launch robstride_ros2 rs_motor_ros2_launch.py

# 3. Start RL control
ros2 run rl_sar rl_real_my_dog --ros-args -p rname:=my_dog
```

### Utility Scripts

```bash
# Check motor status
python3 scripts/check_motor.py

# Monitor linear velocity
python3 scripts/watch_lin_vel.py

# Diagnose controller
bash scripts/diagnose_controller.sh
```

### Control Reference

#### Keyboard Controls

| Key | Function |
|-----|----------|
| **State Switching** |
| `Num0` | Passive → GetUp (stand) |
| `Num9` | GetUp → GetDown (lie down) |
| `Num5` | Calf Swing |
| `Num6` | Climb Wall |
| `Num7` | Bridge Crossing |
| `Num8` | Squat |
| `N` | Toggle navigation mode (accept `/cmd_vel`) |
| **Skill Switching** |
| `Num1` | RL Locomotion 1 (basic walking) |
| `Num2` | RL Locomotion 2 |
| `Num3` | RL Locomotion 3 |
| `Num4` | RL Locomotion 4 |
| **Movement** |
| `W` | Forward (x+) |
| `S` | Backward (x-) |
| `A` | Strafe left (y+) |
| `D` | Strafe right (y-) |
| `Q` | Turn left (yaw+) |
| `E` | Turn right (yaw-) |
| `Space` | Reset all movement commands to zero |
| **Motor** |
| `P` | Switch to passive mode (kp=0, kd=8) |
| `R` | Reset robot in simulation |
| **Simulation Only** |
| `Enter` | Toggle Gazebo pause/resume |

#### Gamepad Controls (Standard Mapping)

Requires ROS2 `joy` node:

```bash
sudo apt install ros-humble-joy
ros2 run joy joy_node
```

Standard gamepad button mapping (tested with XBOX/PS4 controllers):

| Gamepad Input | Function |
|--------------|----------|
| **State Switching** |
| `A` | GetUp (stand up) |
| `B` | GetDown (lie down) |
| `X` | Toggle navigation mode |
| **Skill Switching** |
| `RB + D-Pad Up` | RL Locomotion 1 (basic walking) |
| `RB + D-Pad Down` | RL Locomotion 2 |
| `RB + D-Pad Left` | RL Locomotion 3 |
| `RB + D-Pad Right` | RL Locomotion 4 |
| `RT + D-Pad Up` | Climb Wall |
| `RT + D-Pad Down` | Bridge Crossing |
| **Motor Control** |
| `LB + A` | Motor enable (executes fix.sh) |
| `LB + B` | Motor disable (executes robstride_off.sh) |
| `LB + X` | Passive mode |
| `LB + RB` | Emergency stop |
| **System** |
| `LT + B` | System shutdown (executes poweroff.sh) |
| **Movement** |
| Left Stick Y-axis | Forward/Backward |
| Left Stick X-axis | Strafe Left/Right |
| Right Stick X-axis | Turn Left/Right |

## Pretrained Policies

Multiple policy versions are available in `policy/my_dog/`:

| Version | Directory |
|---------|-----------|
| robot_lab | `policy/my_dog/robot_lab/` |
| robot_lab2 | `policy/my_dog/robot_lab2/` |
| robot_lab3 | `policy/my_dog/robot_lab3/` |
| robot_lab4 | `policy/my_dog/robot_lab4/` |

> To train policies, use [robot_lab](https://github.com/fan-ziqi/robot_lab) project. Export the trained model as ONNX and place it in the corresponding directory.

## Project Structure

```
fzsd2026/
├── src/
│   ├── rl_sar/                    # Core framework (stripped down)
│   │   ├── src/rl_real_my_dog.cpp # Real robot deployment
│   │   ├── src/rl_sim.cpp         # Gazebo simulation
│   │   ├── fsm_robot/             # State machines
│   │   └── launch/                # Launch files
│   ├── rl_sar_zoo/                # Robot descriptions
│   ├── robstride_ros2/            # ★ RobStride motor driver
│   ├── robot_joint_controller/    # ★ Joint controller
│   └── robot_msgs/                # ★ Custom messages
├── policy/my_dog/                 # Pretrained policies
├── scripts/                       # Utility scripts
├── build.sh                       # Build script
└── setup.sh                       # CAN configuration
```

## License

This project is licensed under the [Apache License 2.0](LICENSE), same as the original project.

## Acknowledgements

- [rl_sar](https://github.com/fan-ziqi/rl_sar) — Core SAR framework
- [robot_lab](https://github.com/fan-ziqi/robot_lab) — IsaacLab training framework
- Original project contributors (see [CONTRIBUTORS.md](CONTRIBUTORS.md))

---

> **Disclaimer: User acknowledges that all risks and consequences arising from using this code shall be solely borne by the user, the author assumes no liability for any direct or indirect damages, and proper safety measures must be implemented prior to operation.**
