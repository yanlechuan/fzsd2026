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
- **自定义动作**：蹲起、爬墙、过断桥等动作序列

## 与原版 rl_sar 的差异

本项目基于 [rl_sar](https://github.com/fan-ziqi/rl_sar) (v4.0.0) 进行了大量精简和定制：

### ✨ 新增
| 项目 | 说明 |
|------|------|
| `src/robstride_ros2/` | **RobStride 电机 ROS2 驱动包** — 多线程多CAN架构，详见下方优化说明 |
| `src/robot_joint_controller/` | **自定义关节控制器** — 适配 my_dog 的 ROS2 control 硬件接口 |
| `src/robot_msgs/` | **自定义消息包** — MotorState, RobotCommand, RobotState, IMU 等 |
| `src/rl_sar/src/rl_real_my_dog.cpp` | **实机部署程序** — my_dog 专用状态机与通信 |
| `src/rl_sar/fsm_robot/fsm_my_dog.hpp` | **my_dog 状态机** — 蹲起/爬墙/过断桥等动作序列 |
| `policy/my_dog/` | **预训练策略** — robot_lab 多个版本 |
| `src/rl_sar_zoo/my_dog_description/` | **my_dog 机器人模型** — URDF/XACRO/MJCF |
| `scripts/` | **辅助脚本** — check_motor, watch_lin_vel, diagnose_controller 等 |
| `setup.sh` | **CAN 接口一键配置脚本** |

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

| 对比维度 | 官方样例 | 本项目 | 提升 |
|---------|---------|--------|------|
| 控制电机数 | 1 个 | 12 个 | 🚀 12x |
| CAN 总线 | 1 条 | 4 条并联 | 🚀 4x |
| 架构 | 单线程串行 | 每条 CAN 独立线程 | 🚀 |
| 参数化 | 硬编码 | ROS2 参数动态配置 | 🚀 |
| 状态发布 | ❌ 无 | 400 Hz 实时发布 | 🚀 新增 |

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

| 指标 | 官方版 | 本项目 | 说明 |
|------|-------|--------|------|
| 控制频率 | 16.7 Hz | **416.4 Hz** 🚀 | 官方远低于 RL 策略需求 (50~200Hz) |
| 命令吞吐 | 200 cmd/s | **1249 cmd/s** 🚀 | 本项目吞吐量 **6.3 倍** |
| 丢包后行为 | 永久阻塞死锁 ✅ | 10ms 超时安全恢复 ✅ | 本项目 55 次丢包 0 次卡死 |
| 平行扩展 | 1 条 CAN 串行 12 电机 | 4 条 CAN × 3 电机并行 | 处理器越弱优势越明显 |

> 处理器性能越低，多线程并行架构的优势越显著。官方版在低端 CPU 上因单线程串行处理大量电机，加上 `recv` 无超时的死锁风险，几乎无法满足实时性要求。

## 硬件平台

| 组件 | 说明 |
|------|------|
| 主控 | NVIDIA Jetson (Orin 系列) |
| 电机 | RobStride 关节电机 × 12 |
| 通信 | CAN 总线（4路 can0~can3） |
| 关节 | 12 DOF（4腿 × 3关节：髋/大腿/小腿） |
| IMU | hipnuc IMU |
| 系统 | Ubuntu 22.04 + ROS2 Humble |

## 快速开始

### 系统依赖

```bash
sudo apt install cmake g++ build-essential libyaml-cpp-dev libeigen3-dev \
  libboost-all-dev libspdlog-dev libfmt-dev libtbb-dev liblcm-dev

# ROS2 相关
sudo apt install ros-humble-control-toolbox ros-humble-hardware-interface \
  ros-humble-controller-interface ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster

# Gazebo
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control

# 其他
sudo apt install ros-humble-teleop-twist-keyboard ros-humble-ros2-control \
  ros-humble-ros2-controllers ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui ros-humble-xacro
```

### 下载第三方库

```bash
bash scripts/download_inference_runtime.sh
bash scripts/download_mujoco.sh
bash scripts/download_robot_descriptions.sh
```

### 编译

```bash
./build.sh
source install/setup.bash
```

> 仅编译特定包：`./build.sh package1 package2`
>
> 清理构建：`./build.sh -c`
>
> 仅编译实机代码（禁用 ROS）：`./build.sh -m`

## 使用

### Gazebo 仿真

```bash
# 启动仿真
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

### 工具脚本

```bash
# 检查电机状态
python3 scripts/check_motor.py

# 查看线速度
python3 scripts/watch_lin_vel.py

# 诊断控制器
bash scripts/diagnose_controller.sh
```

### 控制方式

#### 键盘控制

| 按键 | 功能 |
|------|------|
| **状态切换** |
| `Num0` | 被动状态 → 站立 (GetUp) |
| `Num9` | 站立 → 趴下 (GetDown) |
| `Num5` | 小腿摆动 (CalfSwing) |
| `Num6` | 爬墙动作 (ClimbWall) |
| `Num7` | 过断桥动作 (Bridge) |
| `Num8` | 蹲起 (Squat) |
| `N` | 切换导航模式 (接收 `/cmd_vel`) |
| **技能切换** |
| `Num1` | RL Locomotion 1 (基础行走) |
| `Num2` | RL Locomotion 2 |
| `Num3` | RL Locomotion 3 |
| `Num4` | RL Locomotion 4 |
| **移动控制** |
| `W` | 前进 (x+) |
| `S` | 后退 (x-) |
| `A` | 左移 (y+) |
| `D` | 右移 (y-) |
| `Q` | 左转 (yaw+) |
| `E` | 右转 (yaw-) |
| `Space` | 重置移动命令为零 |
| **电机** |
| `P` | 切换到被动模式 (kp=0, kd=8) |
| `R` | 仿真中重置机器人 |

#### 手柄控制 (标准映射)

使用 ROS2 `joy` 节点，如使用 XB1/PS4 手柄需安装：

```bash
sudo apt install ros-humble-joy
ros2 run joy joy_node
```

| 手柄操作 | 功能 |
|---------|------|
| **状态切换** |
| `A` | 站立 (GetUp) |
| `B` | 趴下 (GetDown) |
| `X` | 切换导航模式 |
| **技能切换** |
| `RB + D-Pad 上` | RL Locomotion 1 (基础行走) |
| `RB + D-Pad 下` | RL Locomotion 2 |
| `RB + D-Pad 左` | RL Locomotion 3 |
| `RB + D-Pad 右` | RL Locomotion 4 |
| `RT + D-Pad 上` | 爬墙 (ClimbWall) |
| `RT + D-Pad 下` | 过断桥 (Bridge) |
| **电机控制** |
| `LB + A` | 电机使能 (执行 fix.sh) |
| `LB + B` | 电机断电 (执行 robstride_off.sh) |
| `LB + X` | 被动模式 |
| `LB + RB` | 紧急停止 |
| **系统** |
| `LT + B` | 关机 (执行 poweroff.sh) |
| **移动控制** |
| 左摇杆 Y (上下) | 前进/后退 |
| 左摇杆 X (左右) | 左移/右移 |
| 右摇杆 X (左右) | 左转/右转 |
```

## 预训练策略

多个版本的策略文件位于 `policy/my_dog/`：

| 版本 | 目录 |
|------|------|
| robot_lab | `policy/my_dog/robot_lab/` |
| robot_lab2 | `policy/my_dog/robot_lab2/` |
| robot_lab3 | `policy/my_dog/robot_lab3/` |
| robot_lab4 | `policy/my_dog/robot_lab4/` |

> 如需训练策略，请使用 [robot_lab](https://github.com/fan-ziqi/robot_lab) 项目。训练后导出 ONNX 模型放入对应目录即可。

## 项目目录

```
fzsd2026/
├── src/
│   ├── rl_sar/                    # 核心框架（精简版）
│   │   ├── src/rl_real_my_dog.cpp # 实机部署
│   │   ├── src/rl_sim.cpp         # Gazebo 仿真
│   │   ├── fsm_robot/             # 状态机
│   │   └── launch/                # 启动文件
│   ├── rl_sar_zoo/                # 机器人模型
│   ├── robstride_ros2/            # ★ RobStride 电机驱动
│   ├── robot_joint_controller/    # ★ 关节控制器
│   └── robot_msgs/                # ★ 自定义消息
├── policy/my_dog/                 # 预训练策略
├── scripts/                       # 工具脚本
├── build.sh                       # 构建脚本
├── setup.sh                       # CAN 配置脚本
└── 额外依赖.md
```

## 许可证

本项目沿用原项目的 [Apache License 2.0](LICENSE)。

## 致谢

- [rl_sar](https://github.com/fan-ziqi/rl_sar) — 提供核心 SAR 框架
- [robot_lab](https://github.com/fan-ziqi/robot_lab) — IsaacLab 训练框架
- 原项目所有贡献者（详见 [CONTRIBUTORS.md](CONTRIBUTORS.md)）

---

> **免责声明：使用者确认使用本代码产生的所有风险及后果均由使用者自行承担，作者不承担任何直接或间接责任，操作前必须确保已采取充分安全防护措施。**

```bash
./build.sh -mj  # or ./build.sh --mujoco
```

详细的使用说明可以通过`./build.sh -h`查看

```bash
Usage: ./build.sh [OPTIONS] [PACKAGE_NAMES...]

Options:
  -c, --clean    Clean workspace (remove symlinks and build artifacts)
  -m, --cmake    Build using CMake (for hardware deployment only)
  -mj,--mujoco   Build with MuJoCo simulator support (CMake only)"
  -h, --help     Show this help message

Examples:
  ./build.sh                    # Build all ROS packages
  ./build.sh package1 package2  # Build specific ROS packages
  ./build.sh -c                 # Clean all symlinks and build artifacts
  ./build.sh --clean package1   # Clean specific package and build artifacts
  ./build.sh -m                 # Build with CMake for hardware deployment
  ./build.sh -mj                # Build with CMake and MuJoCo simulator support
```

> [!TIP]
> 如果 catkin build 报错: `Unable to find either executable 'empy' or Python module 'em'`, 在`catkin build` 之前执行 `catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3`

## 运行

下文中使用 **\<ROBOT\>/\<CONFIG\>** 代替表示不同的环境，如 `go2/himloco` 、 `go2w/robot_lab`。

运行前请将训练好的pt模型文件拷贝到`rl_sar/src/rl_sar/policy/<ROBOT>/<CONFIG>`中，并配置`<ROBOT>/<CONFIG>/config.yaml`和`<ROBOT>/base.yaml`中的参数。

### 仿真

#### Gazebo

打开一个终端，启动gazebo仿真环境

```bash
# ROS1
source devel/setup.bash
roslaunch rl_sar gazebo.launch rname:=<ROBOT>

# ROS2
source install/setup.bash
ros2 launch rl_sar gazebo.launch.py rname:=<ROBOT>
```

打开一个新终端，启动控制程序

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_sim

# ROS2
source install/setup.bash
ros2 run rl_sar rl_sim
```

> [!TIP]
> Ubuntu22.04中若启动Gazebo后看不到机器人，则是机器人初始化到了视野范围外，启动rl_sim后会自动重置机器人位置。若机器人在站立过程中翻倒，请使用键盘`R`或手柄`RB+Y`重置机器人环境。

如果第一次启动Gazebo无法打开则需要下载模型包

```bash
git clone https://github.com/osrf/gazebo_models.git ~/.gazebo/models
```

#### Mujoco

```bash
./cmake_build/bin/rl_sim_mujoco <ROBOT> <SCENE>
# Example: ./cmake_build/bin/rl_sim_mujoco g1 scene_29dof
```

### 使用手机网页控制 (实验性)

安装依赖

```bash
sudo apt install ros-${ROS_DISTRO}-rosbridge-suite
sudo apt install ros-${ROS_DISTRO}-web-video-server

# 如果您使用的ROS2版本不是Humble、Jazz或Rolling，需要从源码编译 `web_video_server`
cd <your_ros2_workspace>/src
git clone https://github.com/RobotWebTools/web_video_server.git
cd <your_ros2_workspace>
colcon build --packages-select web_video_server
```

在机器人上运行 rosbridge 和 web_video_server

```bash
# ROS1
roslaunch rosbridge_server rosbridge_websocket.launch
rosrun web_video_server web_video_server

# ROS2
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
ros2 run web_video_server web_video_server
```

访问 [http://robot.robotsfan.com/](http://robot.robotsfan.com/)， 填写IP地址和端口，检查右上角的设置界面，然后连接机器人。进入控制页面后，将屏幕水平放置，点击左上角的全屏按钮，即可使用手机浏览器控制机器人！

### 使用手柄或键盘控制

|手柄控制|键盘控制|功能描述|
|---|---|---|
|**基础**|||
|A|Num0|让机器人从程序开始运行时的姿态以位控插值运动到`base.yaml`中定义的`default_dof_pos`|
|B|Num9|让机器人从当前位置以位控插值运动到程序开始运行时的姿态|
|X|N|切换导航模式 (导航模式屏蔽速度命令，接收`cmd_vel`话题)|
|Y|N/A|N/A|
|**仿真**|||
|RB+Y|R|重置Gazebo环境 (让摔倒的机器人站起来)|
|RB+X|Enter|切换Gazebo运行/停止 (默认为运行状态)|
|**电机**|||
|LB+A|M|N/A (推荐设置为电机使能)|
|LB+B|K|N/A (推荐设置为电机失能)|
|LB+X|P|电机Passive模式 (`kp=0, kd=8`)|
|LB+RB|N/A|N/A (推荐设置为急停保护)|
|**技能**|||
|RB+DPadUp|Num1|基础Locomotion|
|RB+DPadDown|Num2|技能2|
|RB+DPadLeft|Num3|技能3|
|RB+DPadRight|Num4|技能4|
|LB+DPadUp|Num5|技能5|
|LB+DPadDown|Num6|技能6|
|LB+DPadLeft|Num7|技能7|
|LB+DPadRight|Num8|技能8|
|**移动**|||
|LY轴|W/S|前后移动 (X轴)|
|LX轴|A/D|左右移动 (Y轴)|
|RX轴|Q/E|偏航旋转 (Yaw)|
|N/A(松开摇杆)|Space|将所有控制指令设置为零|

### 真实机器人

<details>

<summary>Unitree A1（点击展开）</summary>

与Unitree A1连接可以使用无线与有线两种方式

- 无线：连接机器人发出的Unitree开头的WIFI **（注意：无线连接可能会出现丢包断联甚至失控，请注意安全）**
- 有线：用网线连接计算机和机器人的任意网口，配置计算机地址为192.168.123.162，子网掩码255.255.255.0

新建终端，启动控制程序

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_real_a1

# ROS2
source install/setup.bash
ros2 run rl_sar rl_real_a1

# CMake
./cmake_build/bin/rl_real_a1
```

</details>

<details>

<summary>Unitree Go2/Go2W/G1(29dofs)（点击展开）</summary>

#### 网线连接

用网线的一端连接Go2/Go2W/G1(29dofs)机器人，另一端连接你的电脑，并开启电脑的 USB Ethernet 后进行配置。机器狗机载电脑的 IP 地地址为 `192.168.123.161`，故需将电脑 USB Ethernet 地址设置为与机器狗同一网段，如在 Address 中输入 `192.168.123.222` (`222`可以改成其他)。

通过`ifconfig`命令查看123网段的网卡名字，如`enxf8e43b808e06`，下文用 \<YOUR_NETWORK_INTERFACE\> 代替

Go2:

新建终端，启动控制程序。如果控制Go2W，需要在命令后加`wheel`，否则留空。

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_real_go2 <YOUR_NETWORK_INTERFACE> [wheel]

# ROS2
source install/setup.bash
ros2 run rl_sar rl_real_go2 <YOUR_NETWORK_INTERFACE> [wheel]

# CMake
./cmake_build/bin/rl_real_go2 <YOUR_NETWORK_INTERFACE> [wheel]
```

G1(29dofs):

开机后将机器人吊起来，按L2+R2进入调试模式，然后新建终端，启动控制程序。

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_real_g1 <YOUR_NETWORK_INTERFACE>

# ROS2
source install/setup.bash
ros2 run rl_sar rl_real_g1 <YOUR_NETWORK_INTERFACE>

# CMake
./cmake_build/bin/rl_real_g1 <YOUR_NETWORK_INTERFACE>
```

#### 在机载Jetson中部署

使用网线连接电脑和机器人，登陆Jetson主机，密码123：

```bash
ssh unitree@192.168.123.18
```

将手机连接机器人的USB，开启手机的USB网络共享，拉取代码并使用 `./build.sh -m` 编译，编译成功后运行：

```bash
# Go2:
./cmake_build/bin/rl_real_go2 <YOUR_NETWORK_INTERFACE> [wheel]

# G1(29dofs):
./cmake_build/bin/rl_real_g1 <YOUR_NETWORK_INTERFACE>
```

然后即可拔掉手机和网线，使用遥控器控制。

#### 开机自启动

如需设置开机自启动，可以参考以下流程：

创建一个服务文件

```bash
sudo touch /etc/systemd/system/rl_sar.service
```

写入以下内容，假设rl_sar工程在 `~/rl_sar` 目录下

```
[Unit]
Description=RL SAR Service
After=network.target

[Service]
Type=simple
User=unitree
WorkingDirectory=/home/unitree/rl_sar
ExecStart=/home/unitree/rl_sar/cmake_build/bin/rl_real_go2 eth0 wheel
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

重新加载 systemd 配置：

```bash
sudo systemctl daemon-reload
```

设置开机自动启动：

```bash
sudo systemctl enable rl_sar.service
```

禁用开机自动启动：

```bash
sudo systemctl disable rl_sar.service
```

启动服务：

```bash
sudo systemctl enable rl_sar.service
```

停止服务：

```bash
sudo systemctl stop rl_sar.service
```

重启服务：

```bash
sudo systemctl restart rl_sar.service
```

查看服务日志：

```bash
sudo journalctl -u rl_sar.service -f
```

重启后机机器人会先运行内置站立程序，rl_sar服务启动后则会自动阻尼趴下，随后可以使用遥控器正常控制。

</details>

<details>

<summary>云深处科技 Lite3 (Click to expand)</summary>

Lite3通过无线网络进行连接。
(由于一些型号的Lite3没有开放网线接口，需要额外安装，所以有线连接方式暂时没有进行测试)

- 连接Lite3的Wifi，并测试通信状况。我们强烈建议在运行本项目之前，先通过 [Lite3_Motion_SDK](https://github.com/DeepRoboticsLab/Lite3_MotionSDK)进行测试和检查，在确认一切正常后再运行。
 **(注意：无线连接可能会出现丢包断联甚至失控，请注意安全)**

- 确认所使用Lite3的IP地址和本地端口与目标端口号码，并设置 **在 rl_sar/src/rl_real_lite3.cpp的行46-48**中.
- 在Lite3的运动主机中设置 **jy_exe/conf/network.toml**，使其IP地址指向与Lite3同一网段的本机，建立基于UDP的双向通信.

> [!CAUTION]
> **检查关节映射参数<br>检查确认 rl_sar/policy/himloco/config.yaml中的joint mappng参数。在Sim2Sim中使用的默认joint mapping参数与实机部署时的joint mapping是不同的，如果使用错误可能造成机器人错误的行为，带来潜在的硬件损坏和安全风险。**

Lite3也支持使用云深处Retroid手柄控制，详情参见[Deeprobotics Gamepad](https://github.com/DeepRoboticsLab/gamepad)

新建终端，启动控制程序

```bash
# ROS1
source devel/setup.bash
rosrun rl_sar rl_real_lite3

# ROS2
source install/setup.bash
ros2 run rl_sar rl_real_lite3

# CMake
./cmake_build/bin/rl_real_lite3
```

</details>

### 训练执行器网络

下面拿A1举例

1. 取消注释`rl_real_a1.hpp`中最上面的`#define CSV_LOGGER`，你也可以在仿真程序中修改对应部分采集仿真数据用来测试训练过程。
2. 运行控制程序，程序会记录所有数据到`src/rl_sar/policy/<ROBOT>/motor.csv`。
3. 停止控制程序，开始训练执行器网络。注意，下面的路径前均省略了`rl_sar/src/rl_sar/policy/`。
    ```bash
    rosrun rl_sar actuator_net.py --mode train --data a1/motor.csv --output a1/motor.pt
    ```
4. 验证已经训练好的训练执行器网络。
    ```bash
    rosrun rl_sar actuator_net.py --mode play --data a1/motor.csv --output a1/motor.pt
    ```

## 添加你的机器人

下面使用 **\<ROBOT\>/\<CONFIG\>** 代替表示你的机器人环境。你只需要创建或修改下述文件，命名必须跟下面一样。（你可以参考go2w对应的文件）

```yaml
# 你的机器人description
rl_sar/src/rl_sar_zoo/<ROBOT>_description/CMakeLists.txt
rl_sar/src/rl_sar_zoo/<ROBOT>_description/package.ros1.xml
rl_sar/src/rl_sar_zoo/<ROBOT>_description/package.ros2.xml
rl_sar/src/rl_sar_zoo/<ROBOT>_description/xacro/robot.xacro
rl_sar/src/rl_sar_zoo/<ROBOT>_description/xacro/gazebo.xacro
rl_sar/src/rl_sar_zoo/<ROBOT>_description/config/robot_control.yaml
rl_sar/src/rl_sar_zoo/<ROBOT>_description/config/robot_control_ros2.yaml

# 你训练的policy
policy/<ROBOT>/base.yaml  # 此文件中必须遵守实物机器人的关节顺序
policy/<ROBOT>/<CONFIG>/config.yaml
policy/<ROBOT>/<CONFIG>/<POLICY>.pt  # libtorch使用，注意导出jit
policy/<ROBOT>/<CONFIG>/<POLICY>.onnx  # onnxruntime使用

# 机器人的fsm
src/rl_sar/fsm_robot/fsm_<ROBOT>.hpp
src/rl_sar/fsm_robot/fsm_all.hpp

# 你实物机器人的代码
rl_sar/src/rl_sar/src/rl_real_<ROBOT>.cpp  # 可以按需自定义forward()函数以适配您的policy
```

## 贡献

衷心欢迎社区的贡献，以使这个框架更加成熟和对所有人有用。贡献可以是bug报告、功能请求或代码贡献。

[贡献者名单](CONTRIBUTORS.md)

## 引用

如果您使用此代码或其部分内容，请引用以下内容：

```
@software{fan-ziqi2024rl_sar,
  author = {fan-ziqi},
  title = {rl_sar: Simulation Verification and Physical Deployment of Robot Reinforcement Learning Algorithm.},
  url = {https://github.com/fan-ziqi/rl_sar},
  year = {2024}
}
```

## 致谢

本项目使用了以下开源代码库中的部分代码：

- [unitreerobotics/unitree_sdk2-2.0.0](https://github.com/unitreerobotics/unitree_sdk2/tree/2.0.0)
- [unitreerobotics/unitree_legged_sdk-v3.2](https://github.com/unitreerobotics/unitree_legged_sdk/tree/v3.2)
- [unitreerobotics/unitree_guide](https://github.com/unitreerobotics/unitree_guide)
- [unitreerobotics/unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)
- [google-deepmind/mujoco-3.2.7](https://github.com/google-deepmind/mujoco)
- [mertgungor/unitree_model_control](https://github.com/mertgungor/unitree_model_control)
- [Improbable-AI/walk-these-ways](https://github.com/Improbable-AI/walk-these-ways)
- [ccrpRepo/RoboMimic_Deploy](https://github.com/ccrpRepo/RoboMimic_Deploy)
- [Deeprobotics/Lite3_Motion_SDK](https://github.com/DeepRoboticsLab/Lite3_MotionSDK)
- [chengyangkj/ROS_Flutter_Gui_App](https://github.com/chengyangkj/ROS_Flutter_Gui_App)
