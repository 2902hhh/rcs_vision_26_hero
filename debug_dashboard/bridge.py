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
udp_packet_count = 0

# 尝试导入 websockets
try:
    import websockets
except ImportError:
    print("正在安装 websockets 模块...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "websockets"])
    import websockets


async def ws_handler(websocket):
    """处理新的 WebSocket 连接"""
    ws_clients.add(websocket)
    try:
        remote = websocket.remote_address
    except Exception:
        remote = "unknown"
    print(f"[WS] 新连接: {remote}")
    try:
        async for _ in websocket:
            pass  # 只需要接收连接, 不需要处理客户端消息
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        ws_clients.discard(websocket)
        print(f"[WS] 断开: {remote}")


class UDPProtocol(asyncio.DatagramProtocol):
    """使用 asyncio 原生 DatagramProtocol 接收 UDP，比 sock_recv 更可靠"""

    def __init__(self, loop):
        self.loop = loop

    def datagram_received(self, data, addr):
        global udp_packet_count
        udp_packet_count += 1

        msg = data.decode("utf-8", errors="replace")

        # 验证JSON格式
        try:
            json.loads(msg)
        except json.JSONDecodeError:
            return

        # 每收到前几个包打印一下（帮助调试）
        if udp_packet_count <= 3:
            preview = msg[:120] + ("..." if len(msg) > 120 else "")
            print(f"[UDP] 收到第 {udp_packet_count} 个包 ({len(data)} bytes): {preview}")

        # 广播给所有连接的 WebSocket 客户端
        if ws_clients:
            dead = set()
            for client in ws_clients.copy():
                try:
                    self.loop.create_task(client.send(msg))
                except Exception:
                    dead.add(client)
            ws_clients -= dead

    def error_received(self, exc):
        print(f"[UDP] 错误: {exc}")


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
    loop = asyncio.get_running_loop()

    # 启动 HTTP 服务器 (在单独线程中)
    http_thread = threading.Thread(target=start_http_server, daemon=True)
    http_thread.start()

    # 启动 WebSocket 服务器 (兼容新旧版本 websockets)
    try:
        ws_server = await websockets.serve(ws_handler, WS_HOST, WS_PORT)
    except AttributeError:
        ws_server = await websockets.server.serve(ws_handler, WS_HOST, WS_PORT)
    print(f"[WS] WebSocket 服务器: ws://localhost:{WS_PORT}")

    # 启动 UDP 监听 (使用 asyncio 原生 DatagramProtocol)
    transport, protocol = await loop.create_datagram_endpoint(
        lambda: UDPProtocol(loop),
        local_addr=(UDP_HOST, UDP_PORT),
    )
    print(f"[UDP] 监听 {UDP_HOST}:{UDP_PORT}")

    print(f"\n{'='*55}")
    print(f"  Debug Monitor 已就绪!")
    print(f"  在 VS Code 中打开 Simple Browser:")
    print(f"    http://localhost:{HTTP_PORT}/index.html")
    print(f"{'='*55}\n")
    print(f"[等待] 等待 C++ 程序发送 UDP 数据...\n")

    # 保持运行
    try:
        await asyncio.Future()  # 永不结束
    finally:
        transport.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[Bridge] 已停止")
