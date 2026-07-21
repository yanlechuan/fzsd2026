# rl_sar — 四足机器人强化学习控制

基于强化学习的四足机器人运动控制框架，支持 Gazebo 仿真、MuJoCo 仿真及真实机器人部署。

## 快速启动

```bash
# 编译
./build.sh

# Gazebo 仿真
ros2 run rl_sar rl_sim

# 真实机器人
ros2 run rl_sar rl_real_my_dog
```

## FSM 状态机


| 状态                                | 键盘       | 手柄              | 说明                                  |
| ----------------------------------- | ---------- | ----------------- | ------------------------------------- |
| `RLFSMStatePassive`                 | `P`        | LB + X            | 卸力保护（任意状态可跳入）            |
| `RLFSMStateGetUp`                   | `0`        | A                 | 站立                                  |
| `RLFSMStateRLLocomotion`            | `1`        | RB + DPad↑       | RL 运动控制 — 策略1 (`robot_lab`)     |
| `RLFSMStateRLLocomotion2`           | `2`        | RB + DPad↓       | RL 运动控制 — 策略2 (`robot_lab2`)    |
| `RLFSMStateRLLocomotion3`           | `3`        | RB + DPad←       | RL 运动控制 — 策略3 (`robot_lab3`)    |
| `RLFSMStateRLLocomotion4`           | `4`        | RB + DPad→       | RL 运动控制 — 策略4 (`robot_lab4`)    |
| `RLFSMStateCalfSwing`               | `5`        | —                | 小腿标定                              |
| `RLFSMStateClimbWall`               | `6`        | RT + DPad↑       | 爬墙位控序列                          |
| `RLFSMStateBridge`                  | `7`        | RT + DPad↓       | 过断桥位控序列                        |
| `RLFSMStateSquat`                   | `8`        | —                | 下蹲 → 保持 → 站起                  |
| `RLFSMStateGetDown`                 | `9`        | B                 | 趴下                                  |
| （导航模式切换）                    | `N`        | X                 | 速度控制权交给 `/cmd_vel`             |
| （执行 `fix.sh`）                   | —         | LB + A            | 电机使能                              |
| （执行 `robstride_off.sh`）         | —         | LB + B            | 电机失能                              |
| （执行 `poweroff.sh`）              | —         | LT + B            | 系统关机（LT 扳机锁 > 50%）           |

## 导航自动控制接口

外部导航系统通过以下 4 个 ROS2 topic 实现"导航→动作→导航"闭环：

---

### 1. `/nav_mode` `(std_msgs/String)` ← Sub · 导航模式开关

**发送值**（你必须发这三个之一）：


| 发送       | 含义                                              |
| ---------- | ------------------------------------------------- |
| `"ON"`     | 开启导航模式，速度来源从键盘/手柄切换为`/cmd_vel` |
| `"OFF"`    | 关闭导航模式，恢复键盘/手柄控制                   |
| `"TOGGLE"` | 切换当前模式（等效键盘`N` / 手柄 `X`）            |

**注意**：此 topic 只切换速度来源，不触发 FSM 状态切换。进入到 RLLocomotion 状态后才能让 `/cmd_vel` 生效。

---

### 2. `/fsm_state_request` `(std_msgs/String)` ← Sub · 状态切换请求

**发送值**（你必须选一个已注册的状态名）：


| 发送                        | 含义                      | 从哪个状态可跳转                                 |
| --------------------------- | ------------------------- | ------------------------------------------------ |
| `"RLFSMStatePassive"`       | 卸力保护（最高优先级）    | 任意状态                                         |
| `"RLFSMStateGetUp"`         | 站立                      | Passive、GetDown、CalfSwing、RLLocomotion、Squat |
| `"RLFSMStateGetDown"`       | 趴下                      | GetUp、CalfSwing、RLLocomotion                   |
| `"RLFSMStateRLLocomotion"`  | 策略1步行（`robot_lab`）  | GetUp、CalfSwing、其他RLLocomotion               |
| `"RLFSMStateRLLocomotion2"` | 策略2步行（`robot_lab2`） | GetUp、CalfSwing、其他RLLocomotion               |
| `"RLFSMStateRLLocomotion3"` | 策略3步行（`robot_lab3`） | GetUp、CalfSwing、其他RLLocomotion               |
| `"RLFSMStateRLLocomotion4"` | 策略4步行（`robot_lab4`） | GetUp、CalfSwing、其他RLLocomotion               |
| `"RLFSMStateClimbWall"`     | 爬墙位控序列              | 任意可跳转状态                                   |
| `"RLFSMStateBridge"`        | 过断桥位控序列            | 任意可跳转状态                                   |
| `"RLFSMStateSquat"`         | 下蹲→保持→站起序列      | Passive、GetUp、RLLocomotion                     |
| ❌`"RLFSMStateCalfSwing"`   | 小腿标定                  | **不可通过导航进入**（安全限制）                 |

**注意**：

- 未注册或非法状态名会被静默忽略
- `/fsm_state_request` 只在**导航模式开启**（`navigation_mode=true`）时生效，与键盘 `N` 无关时也需要先发 `/nav_mode "ON"`
- 实际上是：`TryNavStateChange` 只检查 `navigation_mode` 标志，不要求当前控制源是 `/cmd_vel`

---

### 3. `/cmd_vel` `(geometry_msgs/Twist)` ← Sub · 速度指令

**仅在导航模式开启 + 处于 RLLocomotion 状态时生效。**


| 字段        | 类型      | 含义                | 坐标系    |
| ----------- | --------- | ------------------- | --------- |
| `linear.x`  | `float64` | 前进/后退速度 (m/s) | 机体前方  |
| `linear.y`  | `float64` | 左/右平移速度 (m/s) | 机体侧方  |
| `angular.z` | `float64` | 偏航角速度 (rad/s)  | 机体 Z 轴 |

`linear.z`、`angular.x`、`angular.y` 三个字段不使用。

---

### 4. `/fsm_state` `(std_msgs/String)` → **Pub** · 状态反馈

**你会收到的值**（状态变化时发布，一帧延迟）：


| 收到的值                    | 含义                                                                    |
| --------------------------- | ----------------------------------------------------------------------- |
| `"RLFSMStatePassive"`       | 已进入卸力状态                                                          |
| `"RLFSMStateGetUp"`         | 已进入站立状态（Squat 完成、GetDown 途中手动站起等都会进入此状态）      |
| `"RLFSMStateGetDown"`       | 正在趴下（完成自动切 Passive，不会再发 GetUp）                          |
| `"RLFSMStateRLLocomotion"`  | 已进入策略1运动态，可接收`/cmd_vel`                                     |
| `"RLFSMStateRLLocomotion2"` | 已进入策略2运动态，可接收`/cmd_vel`                                     |
| `"RLFSMStateRLLocomotion3"` | 已进入策略3运动态，可接收`/cmd_vel`                                     |
| `"RLFSMStateRLLocomotion4"` | 已进入策略4运动态，可接收`/cmd_vel`                                     |
| `"RLFSMStateSquat"`         | 已进入下蹲序列（下蹲→保持→站起，约`down+hold+up` 秒后自动切回 GetUp） |
| `"RLFSMStateClimbWall"`     | 已进入爬墙位控序列                                                    |
| `"RLFSMStateBridge"`        | 已进入过断桥位控序列                                                  |
| `"RLFSMStateCalfSwing"`     | 已进入小腿标定（不可通过导航进入，仅键盘/手柄触发）                   |

**发布时机**：`RobotControl()` 每个控制周期检查，仅在状态名变化时发布一次。

---

### 典型导航流程

```
外部决策端                               rl_sar
    │
    ├─ /nav_mode "ON" ─────────────────▶ 开启导航模式
    │
    ├─ /fsm_state_request               ─▶ 0→站立→运动态
    │     "RLFSMStateRLLocomotion"
    │  ◀── /fsm_state                   确认：已进入运动态
    │     "RLFSMStateRLLocomotion"
    │
    ├─ /cmd_vel                         导航到目标点
    │     linear.x=0.3, angular.z=0.1
    │  ...持续发送...
    │
    ├─ /fsm_state_request               ─▶ 到达，执行下蹲
    │     "RLFSMStateSquat"
    │  ◀── /fsm_state                   进入 Squat
    │     "RLFSMStateSquat"
    │  ◀── /fsm_state                   Squat 完成，自动回到站立
    │     "RLFSMStateGetUp"
    │
    ├─ /fsm_state_request               ─▶ 重新进入运动态
    │     "RLFSMStateRLLocomotion"
    │  ◀── /fsm_state                   确认
    │     "RLFSMStateRLLocomotion"
    │
    ├─ /cmd_vel ... ───────────────────▶ 继续下一段导航
```

### 注意事项

- **状态反馈存在一帧延迟**：`/fsm_state` 在 `Enter()` 后的下一个控制周期发布
- **Passive 可中断任何状态**：`/fsm_state_request "RLFSMStatePassive"` 在所有状态下都立即生效
- **CalfSwing 不可通过导航进入**：`TryNavStateChange` 明确排除该状态，仅键盘/手柄可触发
- **MuJoCo 暂不支持导航**：`/cmd_vel` 接管逻辑在 MuJoCo 版本中被注释
- **导航模式与状态切换相互独立**：`/nav_mode` 只控制速度来源；`/fsm_state_request` 只触发状态切换。两者可以独立使用

## 目录结构

```
src/rl_sar/
├── fsm_robot/        # FSM 状态机定义 (my_dog)
├── include/          # 头文件 (rl_sim, rl_real_my_dog, rl_sim_mujoco)
├── src/              # 实现文件
├── launch/           # 启动文件
├── library/          # 核心库 (FSM, RL SDK, 推理运行时)
├── worlds/           # Gazebo 世界文件
└── test/             # 单元测试
```

## 依赖

- ROS2 Humble / ROS1 Noetic
- Gazebo / MuJoCo
- yaml-cpp
- TBB
- ONNX Runtime (推理运行时)
