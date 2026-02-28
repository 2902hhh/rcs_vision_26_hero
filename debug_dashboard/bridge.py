#!/usr/bin/env python3
"""
sp_vision Debug Monitor - WebSocket Bridge
监听 UDP 9870 端口的 JSON 数据, 通过 WebSocket 转发给 Web 仪表盘。

用法:
  python3 debug_dashboard/bridge.py

依赖:
  pip3 install websockets   (如果没有的话)
"""

import asyncio
import json
import socket
import os
import sys
import http.server
import threading

# 配置
UDP_HOST = "0.0.0.0"
UDP_PORT = 9870
WS_HOST = "0.0.0.0"
WS_PORT = 9871
HTTP_PORT = 9872

# 全局 WebSocket 客户端集合
ws_clients = set()

# 尝试导入 websockets
try:
    import websockets
    import websockets.server
except ImportError:
    print("正在安装 websockets 模块...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "websockets"])
    import websockets
    import websockets.server


async def ws_handler(websocket):
    """处理新的 WebSocket 连接"""
    ws_clients.add(websocket)
    remote = websocket.remote_address
    print(f"[WS] 新连接: {remote}")
    try:
        async for _ in websocket:
            pass  # 只需要接收连接, 不需要处理客户端消息
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        ws_clients.discard(websocket)
        print(f"[WS] 断开: {remote}")


async def udp_listener(loop):
    """监听 UDP 数据并广播给所有 WebSocket 客户端"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((UDP_HOST, UDP_PORT))
    sock.setblocking(False)
    print(f"[UDP] 监听 {UDP_HOST}:{UDP_PORT}")

    while True:
        try:
            data = await loop.sock_recv(sock, 65536)
            if not data:
                continue

            msg = data.decode("utf-8", errors="replace")

            # 验证JSON格式
            try:
                json.loads(msg)
            except json.JSONDecodeError:
                continue

            # 广播给所有连接的 WebSocket 客户端
            if ws_clients:
                dead = set()
                for client in ws_clients.copy():
                    try:
                        await client.send(msg)
                    except Exception:
                        dead.add(client)
                ws_clients -= dead

        except Exception as e:
            print(f"[UDP] 错误: {e}")
            await asyncio.sleep(0.01)


def start_http_server():
    """启动一个简单的 HTTP 服务器来提供 HTML 页面"""
    dashboard_dir = os.path.dirname(os.path.abspath(__file__))

    class QuietHandler(http.server.SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=dashboard_dir, **kwargs)

        def log_message(self, format, *args):
            pass  # 静默HTTP日志

    server = http.server.HTTPServer(("0.0.0.0", HTTP_PORT), QuietHandler)
    print(f"[HTTP] 仪表盘: http://localhost:{HTTP_PORT}/index.html")
    server.serve_forever()


async def main():
    loop = asyncio.get_event_loop()

    # 启动 HTTP 服务器 (在单独线程中)
    http_thread = threading.Thread(target=start_http_server, daemon=True)
    http_thread.start()

    # 启动 WebSocket 服务器
    ws_server = await websockets.server.serve(ws_handler, WS_HOST, WS_PORT)
    print(f"[WS] WebSocket 服务器: ws://localhost:{WS_PORT}")
    print(f"\n{'='*55}")
    print(f"  Debug Monitor 已就绪!")
    print(f"  在 VS Code 中打开 Simple Browser:")
    print(f"    http://localhost:{HTTP_PORT}/index.html")
    print(f"{'='*55}\n")

    # 启动 UDP 监听
    await udp_listener(loop)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[Bridge] 已停止")
