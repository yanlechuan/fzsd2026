#!/bin/bash
# 设置CAN接口
echo "配置CAN接口..."

# 先关闭所有CAN接口（如果已存在）
sudo ip link set can0 down 2>/dev/null
sudo ip link set can1 down 2>/dev/null
sudo ip link set can2 down 2>/dev/null
sudo ip link set can3 down 2>/dev/null
sudo ip link set can4 down 2>/dev/null

# 配置CAN接口参数
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can2 type can bitrate 1000000
sudo ip link set can3 type can bitrate 1000000
sudo ip link set can4 type can bitrate 1000000

# 启动CAN接口
sudo ip link set can0 up
sudo ip link set can1 up
sudo ip link set can2 up
sudo ip link set can3 up
sudo ip link set can4 up

# 设置USB设备权限
sudo chmod 777 /dev/ttyUSB0 2>/dev/null || echo "未找到/dev/ttyUSB0，跳过权限设置"

# 显示配置结果
echo "CAN接口配置完成："
