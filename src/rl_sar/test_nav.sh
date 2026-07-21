#!/bin/bash
echo "=== 1. 开启导航模式 ==="
ros2 topic pub -1 /nav_mode std_msgs/msg/String "data: 'ON'"

sleep 1

echo "=== 2. 切到 RLLocomotion ==="
ros2 topic pub -1 /fsm_state_request std_msgs/msg/String "data: 'RLFSMStateRLLocomotion'"

sleep 3

echo "=== 3. 前进 4 秒 ==="
timeout 4 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" 2>/dev/null

echo "=== 4. 停下，触发 Squat ==="
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{}"
sleep 1.0
ros2 topic pub -1 /fsm_state_request std_msgs/msg/String "data: 'RLFSMStateSquat'"

echo "=== 5. 等待 Squat 完成（约 5 秒）==="
sleep 6

echo "=== 6. 重回运动态，继续前进 ==="
ros2 topic pub -1 /fsm_state_request std_msgs/msg/String "data: 'RLFSMStateRLLocomotion'"
sleep 1
timeout 5 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" 2>/dev/null
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{}"
echo "=== 完成 ==="