"""
零依赖版 viewer —— 只用 Python 标准库，不需要 fastapi / uvicorn
用法：python3 Viewer/viewer_lite.py
然后浏览器打开 http://localhost:8000
数据通路与 viewer.py 完全一致：UDP 9999 → WebSocket → 网页
"""
import base64
import hashlib
import json
import os
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

UDP_PORT  = 9999
HTTP_PORT = 8000
BASE = os.path.dirname(os.path.abspath(__file__))
HTML_PATH = os.path.join(BASE, "Graound_view_2.html")
WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

latest = {"text": None}
clients = []
lock = threading.Lock()


def ws_frame(payload: bytes) -> bytes:
    """服务器 → 浏览器，不掩码"""
    n = len(payload)
    if n < 126:
        return bytes([0x81, n]) + payload
    if n < 65536:
        return bytes([0x81, 126]) + n.to_bytes(2, "big") + payload
    return bytes([0x81, 127]) + n.to_bytes(8, "big") + payload


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path.startswith("/ws"):
            self._ws()
        else:
            self._html()

    def _html(self):
        try:
            with open(HTML_PATH, "rb") as f:
                body = f.read()
        except FileNotFoundError:
            self.send_error(404, "Graound_view_2.html not found")
            return
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _ws(self):
        key = self.headers.get("Sec-WebSocket-Key")
        if not key:
            self.send_error(400)
            return
        acc = base64.b64encode(
            hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
        self.send_response(101)
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", acc)
        self.end_headers()
        print("[WS] 浏览器已连接")

        sock = self.connection
        with lock:
            clients.append(sock)
            pending = latest["text"]
        try:
            if pending:
                sock.sendall(ws_frame(pending.encode("utf-8")))
            while True:                    # 阻塞读到断开为止
                if not sock.recv(4096):
                    break
        except Exception:
            pass
        finally:
            with lock:
                if sock in clients:
                    clients.remove(sock)
            print("[WS] 浏览器断开")


def broadcast(text):
    data = ws_frame(text.encode("utf-8"))
    with lock:
        for c in list(clients):
            try:
                c.sendall(data)
            except Exception:
                if c in clients:
                    clients.remove(c)


def udp_loop():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", UDP_PORT))
    print(f"[UDP] 监听 0.0.0.0:{UDP_PORT}")
    while True:
        data, _ = s.recvfrom(4096)
        try:
            text = data.decode("utf-8")
            json.loads(text)               # 只转发合法 JSON
        except Exception:
            continue
        latest["text"] = text
        broadcast(text)


if __name__ == "__main__":
    threading.Thread(target=udp_loop, daemon=True).start()
    print(f"浏览器打开  http://localhost:{HTTP_PORT}")
    ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler).serve_forever()