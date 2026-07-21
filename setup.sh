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

for if in can0 can1 can2 can3 can4; do
    for i in $(seq 1 20); do
        if ip link show $if >/dev/null 2>&1; then
            break
        fi
        echo "等待 $if 出现... ($i/20)"
        sleep 0.5
    done
    if ! ip link show $if >/dev/null 2>&1; then
        echo "错误: $if 未找到! USB-CAN 适配器可能未就绪"
        exit 1
    fi
done

# 启动CAN接口
sudo ip link set can0 up
sudo ip link set can1 up
sudo ip link set can2 up
sudo ip link set can3 up
sudo ip link set can4 up

# 增大发送队列（避免 No buffer space available）
sudo ip link set can0 txqueuelen 1000
sudo ip link set can1 txqueuelen 1000
sudo ip link set can2 txqueuelen 1000
sudo ip link set can3 txqueuelen 1000
sudo ip link set can4 txqueuelen 1000

# 设置USB设备权限
sudo chmod 777 /dev/ttyUSB0 2>/dev/null || echo "未找到/dev/ttyUSB0，跳过权限设置"

# 显示配置结果
echo "CAN接口配置完成："
