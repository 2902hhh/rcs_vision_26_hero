#!/usr/bin/env bash

# 1. 强制延迟（给系统硬件和网络留出加载时间）
sleep 5

# 2. 绝对路径定义（将 rm 替换为你的实际用户名）
USER_NAME="rm"
PROJECT_DIR="/home/$USER_NAME/Desktop/rcs_vision_26_hero_classic"
SCREEN_BIN="/usr/bin/screen"

# 3. 进入目录，失败则退出
cd "$PROJECT_DIR" || { echo "Directory not found"; exit 1; }

# 4. 创建日志目录
mkdir -p logs

# 5. 启动 Screen 会话
# 修改点：
# -S vision: 给会话命名，方便以后用 screen -r vision 进入
# -L -Logfile: 依然保留日志，但请确保硬盘空间充足
$SCREEN_BIN \
    -L \
    -Logfile "$PROJECT_DIR/logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog" \
    -S vision \
    -d \
    -m \
    bash -lc "./build/standard configs/hero.yaml"

# 6. 给 systemd 一个确定的反馈
echo "Vision program started in screen session 'vision'"