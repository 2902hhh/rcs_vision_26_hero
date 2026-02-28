#!/bin/bash
# sp_vision Debug Monitor 启动脚本
# 用法: ./debug_dashboard/start.sh
# 然后在 VS Code 中: Ctrl+Shift+P -> "Simple Browser: Show" -> http://localhost:9872/index.html

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "================================================="
echo "  sp_vision Debug Monitor"
echo "================================================="
echo ""

# 检查 Python3
if ! command -v python3 &> /dev/null; then
    echo "[错误] 未找到 python3, 请先安装: sudo apt install python3"
    exit 1
fi

# 检查 websockets 模块
if ! python3 -c "import websockets" 2>/dev/null; then
    echo "[提示] 正在安装 websockets 模块..."
    pip3 install websockets
fi

echo "[启动] Python bridge..."
echo "  UDP 监听:  0.0.0.0:9870"
echo "  WebSocket: 0.0.0.0:9871"
echo "  HTTP 面板: http://localhost:9872/index.html"
echo ""
echo "  在 VS Code 中打开面板:"
echo "    Ctrl+Shift+P -> Simple Browser: Show"
echo "    输入: http://localhost:9872/index.html"
echo ""
echo "  按 Ctrl+C 停止"
echo "================================================="
echo ""

python3 "$SCRIPT_DIR/bridge.py"
