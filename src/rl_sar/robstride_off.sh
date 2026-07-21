#!/bin/bash
source /opt/ros/humble/setup.bash
source $HOME/rl_sar/install/setup.bash
cd $HOME/rl_sar


sleep 0.5

tmux send-keys -t robot:0.1 C-c
