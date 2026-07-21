#!/bin/bash
# ========================
# 安装开机自启动服务
# 用法: sudo bash install_autorobot_service.sh
# ========================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
START_SCRIPT="$HOME/start_robot.sh"
SERVICE_NAME="autorobot.service"
SERVICE_PATH="/etc/systemd/system/$SERVICE_NAME"

# 确保 start_robot.sh 存在
if [[ ! -f "$START_SCRIPT" ]]; then
    echo "[ERROR] 未找到 $START_SCRIPT，请先创建启动脚本！"
    exit 1
fi

# 确保 start_robot.sh 可执行
chmod +x "$START_SCRIPT"

# 获取当前用户名
USER_NAME="${SUDO_USER:-$USER}"

echo "[INFO] 当前用户: $USER_NAME"
echo "[INFO] 启动脚本: $START_SCRIPT"

# 写入 systemd 服务文件
cat > "$SERVICE_PATH" << EOF
[Unit]
Description=Auto Robot Startup Service
After=network.target multi-user.target
Wants=network.target

[Service]
Type=forking
User=$USER_NAME
Environment="HOME=$HOME"
Environment="DISPLAY=:0"
Environment="XAUTHORITY=$HOME/.Xauthority"
ExecStartPre=/bin/sleep 3
ExecStart=/bin/bash -l -c '${START_SCRIPT}'
ExecStop=/usr/bin/tmux kill-session -t robot 2>/dev/null
Restart=no
KillMode=mixed
KillSignal=SIGTERM
TimeoutStopSec=5

[Install]
WantedBy=multi-user.target
EOF

echo "[INFO] 服务文件已写入: $SERVICE_PATH"

# 重新加载 systemd
systemctl daemon-reload

# 启用服务
systemctl enable "$SERVICE_NAME"

echo ""
echo "==============================================="
echo "[SUCCESS] 自启动服务安装完成！"
echo ""
echo "常用命令:"
echo "  启动服务:   sudo systemctl start autorobot"
echo "  停止服务:   sudo systemctl stop autorobot"
echo "  查看状态:   sudo systemctl status autorobot"
echo "  查看日志:   journalctl -u autorobot -f"
echo "  禁用自启:   sudo systemctl disable autorobot"
echo "  手动连接:   tmux attach -t robot"
echo "==============================================="
