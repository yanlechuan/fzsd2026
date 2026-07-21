#!/bin/bash
source /opt/ros/humble/setup.bash
source $HOME/rl_sar/install/setup.bash
cd $HOME/rl_sar

# 先配置CAN接口（需要sudo免密）
$HOME/rl_sar/setup.sh

# 直接向 tmux 会话发送命令，无需 attach（避免 "not a terminal" 错误）
tmux send-keys -t robot:0.1 C-c

sleep 0.5
tmux send-keys -t robot:0.1 'ros2 launch rs_motor_ros2 rs_motor_ros2_launch.py' C-m
