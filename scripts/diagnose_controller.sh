#!/bin/bash
echo "=== 详细控制器管理器诊断 ==="
echo

cd ~/robodog_lab/rl_sar
source install/setup.bash

# 1. 启动Gazebo
echo "1. 启动Gazebo（后台运行）..."
ros2 launch rl_sar gazebo.launch.py rname:=go2 > /tmp/gazebo_detailed.log 2>&1 &
GAZEBO_PID=$!
sleep 10

# 2. 查看所有节点和完整信息
echo "2. 所有节点及其服务:"
for node in $(ros2 node list 2>/dev/null); do
    echo "节点: $node"
    ros2 node info $node 2>/dev/null | grep -E "(Services|Subscribers|Publishers)" | head -5
done
echo

# 3. 查看controller_manager服务的详细信息
echo "3. controller_manager服务详细信息:"
ros2 service info /controller_manager/list_controllers 2>/dev/null
echo

# 4. 尝试调用服务并查看响应
echo "4. 调用controller_manager服务并查看响应:"
timeout 3 ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers "{}" 2>&1
echo

# 5. 查看所有controller_manager相关服务
echo "5. 所有controller_manager相关服务:"
ros2 service list | grep -i controller
for service in $(ros2 service list | grep -i controller); do
    echo "  服务: $service"
    ros2 service type $service 2>/dev/null
done
echo

# 6. 查看Gazebo进程
echo "6. Gazebo进程信息:"
ps aux | grep -E "gazebo|gz" | grep -v grep
echo

# 7. 查看Gazebo日志
echo "7. Gazebo日志中的关键信息:"
grep -i "gazebo_ros2_control\|controller_manager\|plugin" /tmp/gazebo_detailed.log | head -20
echo

# 8. 清理
kill $GAZEBO_PID 2>/dev/null
