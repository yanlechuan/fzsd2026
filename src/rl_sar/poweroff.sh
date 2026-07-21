#!/bin/bash
source /opt/ros/humble/setup.bash
source $HOME/rl_sar/install/setup.bash
cd $HOME/rl_sar

# 直接向 tmux 会话发送 Ctrl+C，无需 attach（避免 "not a terminal" 错误）
tmux send-keys -t robot:0.0 C-c
tmux send-keys -t robot:0.1 C-c
tmux send-keys -t robot:0.2 C-c
sleep 2
sudo poweroff