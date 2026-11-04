# Viewer/viewer.py  —— 在本地电脑上运行
import asyncio, json, os, socket, threading
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
import uvicorn

UDP_PORT  = 9999
HTTP_PORT = 8000
BASE = os.path.dirname(os.path.abspath(__file__))
HTML_PATH = os.path.join(BASE, "Graound_view_2.html")

app = FastAPI()
clients = set()
loop = None
last_text = None

@app.get("/")
async def index():
    return FileResponse(HTML_PATH)

@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    global loop
    await ws.accept()
    loop = asyncio.get_running_loop()
    clients.add(ws)
    if last_text:                      # 新开页面立刻有数据
        try: await ws.send_text(last_text)
        except Exception: pass
    try:
        while True:
            await ws.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        clients.discard(ws)

async def _broadcast(text):
    for c in list(clients):
        try:
            await c.send_text(text)
        except Exception:
            clients.discard(c)

def udp_loop():
    global last_text
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", UDP_PORT))
    print(f"[UDP] 监听 0.0.0.0:{UDP_PORT}")
    while True:
        data, _ = s.recvfrom(4096)
        try:
            text = data.decode("utf-8")
            json.loads(text)
        except Exception:
            continue
        last_text = text
        if loop is not None:
            asyncio.run_coroutine_threadsafe(_broadcast(text), loop)

if __name__ == "__main__":
    threading.Thread(target=udp_loop, daemon=True).start()
    print(f"浏览器打开  http://localhost:{HTTP_PORT}")
    uvicorn.run(app, host="0.0.0.0", port=HTTP_PORT, log_level="info")