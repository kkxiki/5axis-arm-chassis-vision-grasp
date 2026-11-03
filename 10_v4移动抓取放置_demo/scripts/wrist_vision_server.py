#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from v4_robot.controller_api import ControllerEventClient
from v4_robot.controller_frame import camera_frames, unpack_envelope


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg: bytes | None = None
        self.timestamp_s = 0.0
        self.frame_count = 0
        self.error = "waiting for first frame"


def subscribe(state: State, endpoint: str, subsystem: str) -> None:
    import zmq
    socket = zmq.Context.instance().socket(zmq.SUB)
    socket.setsockopt(zmq.SUBSCRIBE, b"giant_frame_data")
    socket.connect(endpoint)
    while True:
        try:
            if not socket.poll(2000):
                with state.lock:
                    state.error = "controller frame timeout"
                continue
            topic, payload = unpack_envelope(socket.recv())
            if topic != "giant_frame_data":
                continue
            frame = next((item for item in camera_frames(payload)
                          if item.subsystem_name == subsystem and item.data_type == "img/jpeg"), None)
            if frame is None:
                continue
            with state.lock:
                state.jpeg = frame.data
                state.timestamp_s = time.time()
                state.frame_count += 1
                state.error = ""
        except Exception as exc:
            with state.lock:
                state.error = str(exc)
            time.sleep(0.5)


def make_handler(state: State, stale_s: float):
    class Handler(BaseHTTPRequestHandler):
        def send_json(self, value: dict, status: int = 200):
            body = json.dumps(value, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def send_html(self, value: str, status: int = 200):
            body = value.encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            route = urlparse(self.path)
            with state.lock:
                jpeg = state.jpeg
                timestamp_s = state.timestamp_s
                count = state.frame_count
                error = state.error
            age = time.time() - timestamp_s if timestamp_s else None
            ok = jpeg is not None and age is not None and age <= stale_s
            if route.path == "/health":
                self.send_json({"ok": ok, "camera_id": "wrist", "frame_count": count,
                                "frame_age_s": age, "metric_depth": False, "error": error})
            elif route.path == "/frame.jpg":
                if not ok:
                    self.send_error(503, error or "stale camera frame")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpeg)))
                self.end_headers()
                self.wfile.write(jpeg)
            elif route.path == "/detect":
                label = parse_qs(route.query).get("label", ["bottle"])[0]
                self.send_json({"ok": ok, "detected": False, "label": label,
                                "camera_id": "wrist", "frame_id": "wrist_camera_color_only",
                                "timestamp_s": timestamp_s, "metric_depth": False,
                                "reason": "V4 hand-eye is accepted; aligned metric depth and bottle detector are not wired into this service yet"},
                               200 if ok else 503)
            elif route.path == "/mjpeg":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                last = -1
                try:
                    while True:
                        with state.lock:
                            current, count = state.jpeg, state.frame_count
                        if current is not None and count != last:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                                             str(len(current)).encode() + b"\r\n\r\n" + current + b"\r\n")
                            self.wfile.flush()
                            last = count
                        time.sleep(0.03)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            elif route.path in ("/", "/index.html"):
                self.send_html("""<!doctype html><html lang='zh-CN'><head>
<meta charset='utf-8'><title>Jetson Wrist Camera</title>
<style>body{margin:0;background:#111;color:#eee;font-family:sans-serif;text-align:center}
h2{margin:12px}.live{max-width:96vw;max-height:88vh;border:1px solid #555}</style></head>
<body><h2>V4 手腕相机 · 1280×720 实时画面</h2>
<img id='live' class='live' src='/frame.jpg' alt='wrist camera live stream'>
<script>const image=document.getElementById('live');
image.onload=()=>setTimeout(()=>{image.src='/frame.jpg?t='+Date.now()},100);</script></body></html>""")
            else:
                self.send_json({"service": "v4-wrist-vision", "health": "/health",
                                "frame": "/frame.jpg", "stream": "/mjpeg"}, 404)

        def log_message(self, fmt, *args):
            pass
    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(description="Controller wrist camera to Jetson V4 bridge")
    parser.add_argument("--controller", default="192.168.30.11")
    parser.add_argument("--topic-port", type=int, default=19091)
    parser.add_argument("--subsystem", default="RsCameraSensor#0#0")
    parser.add_argument("--listen-port", type=int, default=8092)
    parser.add_argument("--stale-s", type=float, default=2.0)
    args = parser.parse_args()
    client = ControllerEventClient(args.controller, timeout_s=15.0)
    client.send("StartCameraSensor", camera_sensor=args.subsystem)
    client.send("ShowImageCameraSensor", camera_sensor=args.subsystem)
    state = State()
    endpoint = f"tcp://{args.controller}:{args.topic_port}"
    threading.Thread(target=subscribe, args=(state, endpoint, args.subsystem), daemon=True).start()
    print(f"wrist vision http://0.0.0.0:{args.listen_port} <- {endpoint}/{args.subsystem}", flush=True)
    ThreadingHTTPServer(("0.0.0.0", args.listen_port), make_handler(state, args.stale_s)).serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
