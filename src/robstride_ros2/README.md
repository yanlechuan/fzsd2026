
# rs_motor_ros2

简短说明
- 这是用于 RobStride 电机的 ROS2 包，提供：
  - 订阅 `RobotCommand` 控制多台电机
  - 发布 `RobotState`（包含每台电机状态与 IMU）
  - 支持多 CAN 接口（最多 5 条）与每台电机单独映射到某条 CAN
  - 支持为每台电机指定底层 CAN EID、master_id 与 actuator_type

主要文件
- `src/main.cpp`：节点入口，参数化创建电机实例、订阅/发布逻辑
- `include/motor_ros2/motor_cfg.h`, `src/motor_cfg.cpp`：底层 CAN 驱动与电机控制
- `msg/*.msg`：自定义消息定义（MotorCommand / MotorState / RobotCommand / RobotState）
- `config/params.yaml`：参数示例
- `launch/rs_motor_ros2_launch.py`：launch 文件（加载 `config/params.yaml`）

构建
```bash
colcon build --packages-select rs_motor_ros2
source install/setup.bash
```

配置与参数（`config/params.yaml`）
- `can_ifaces`: CAN 接口数组，例如 `["can0","can1","can2","can3"]`（最多 5）
- `motor_ids`: 逻辑电机 ID 列表（上层索引）
- `motor_iface_map`: 指定每台电机对应 `can_ifaces` 的索引（长度应与 `motor_ids` 一致）
- `motor_can_ids` (可选): 每台电机的底层 CAN EID（用于帧中低 8 位），若空则使用 `motor_ids` 值
- `motor_master_ids` (可选): 每台电机的 master_id（若为空使用全局 `master_id`）
- `motor_actuator_types` (可选): 每台电机的 actuator_type（若为空使用全局 `actuator_type`）
- 其他：`robot_state_topic`, `robot_command_topic`, `imu_topic`, `publish_rate` 等

示例（节选）
```yaml
rs_motor_ros2:
  ros__parameters:s2
    can_ifaces: ["can0","can1","can2","can3"]
    motor_ids: [1,2,3,4,5,6,7,8,9,10,11,12]
    motor_iface_map: [0,0,0,0,1,1,1,1,2,2,3,3]
    motor_can_ids: [0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C]
    master_id: 255
    motor_master_ids: []
    actuator_type: 2
    motor_actuator_types: []
    robot_state_topic: "robot_state"
    robot_command_topic: "robot_command"
    imu_topic: "hipnuc_imu"
    publish_rate: 50
```

CAN 设备与测试
- 开发或测试时可使用虚拟 CAN（vcan）：
  ```bash
  sudo modprobe vcan
  sudo ip link add dev vcan0 type vcan || true
  sudo ip link set up vcan0
  ```
- 若使用 USB-5CAN 等物理设备，请确保对应接口名（例如 `can0..can3`）已创建并 up。
- 监听 CAN 报文（调试）:
  ```bash
  sudo apt install can-utils
  candump can0
  ```
- 发送测试帧：
  ```bash
  cansend can0 123#1122334455667788
  ```

运行节点
- 使用 launch（推荐）：
  ```bash
  ros2 launch rs_motor_ros2 rs_motor_ros2_launch.py
  ```
- 或直接运行并加载参数文件：
  ```bash
  ros2 run rs_motor_ros2 rs_motor_ros2_node --ros-args --params-file config/params.yaml
  ```

验证主题
- 查看 `robot_state`：
  ```bash
  ros2 topic echo /robot_state -n 1
  ```
- 发布测试 `RobotCommand`：
  ```bash
  ros2 topic pub /robot_command rs_motor_ros2/msg/RobotCommand "{motor_command: [{q:0.0, dq:0.0, tau:0.0, kp:1.0, kd:0.1}]}"
  ```

设计说明（简要）
- 逻辑 `motor_id`：供上层使用的索引编号（便于数组映射、配置与调用）。
- 物理 `device_eid`（motor_can_ids）：发送到 CAN 帧的真实编号（用于填低 8 位/target 字段）。
- `master_id`：报文中主控/分组字段，可在多条物理总线上复用，但需避免总线合并后 ID 冲突。

注意事项与安全
- 如果多个主控会在同一条总线上发送命令，请在上层设计仲裁/授权策略，避免冲突。
- 确认每条总线上的 ID 不冲突，使用 `candump` 验证。
- 对真实电机操作前，先在虚拟总线上完成功能验证。

想要改进
- 我可以：
  - 把 `README` 中的示例填充为你真实机器人的配置（把接口名与 ID 填入）；
  - 增加系统级的 watchdog/重连策略示例；
  - 把参数验证与启动 summary 输出格式化为表格。

License
- （未指定）请在需要时补充许可证文件。
# ROS package for RobStride motor control
This routine was reposted by RobStride Dynamics from DR.MuShibo. Sincere gratitude goes to DR.MuShibo for their development and sharing.

### USB2CAN Hardware:Canable
- canable (cantact clone): http://canable.io/ (STM32F042C6)
- 灵足的串口转CAN模块只适用于灵足的上位机，Ubuntu上使用需要额外的canable模块。

## Dependency:
- 注意自己的ros2版本号，自行修改
```shell
sudo apt-get install net-tools
sudo apt-get install can-utils
sudo apt-get install ros-humble-can-msgs
sudo apt-get install ros-humble-socketcan-bridge
```

### Ubuntu
```shell
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev

sudo ip link set can0 type can bitrate 1000000 

sudo ip link set can0 up
sudo ifconfig can0 txqueuelen 100
```

### Launch the launch file for the demo
- 在工作空间中运行如下命令: 
```shell
colcon build 
source ./install/setup.zsh (or bash)
ros2 run rs_motor_ros2 rs_motor_ros2_node
```