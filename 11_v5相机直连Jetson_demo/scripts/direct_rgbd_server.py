#!/usr/bin/env python3
"""V5 direct USB RealSense RGB-D subscriber and HTTP preview server."""
from __future__ import annotations
import argparse, json, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg = None
        self.depth = None
        self.depth_jpeg = None
        self.color_stamp = 0.0
        self.depth_stamp = 0.0
        self.frames = 0
        self.color_encoding = ""
        self.depth_encoding = ""
        self.depth_step = 0
        self.error = "waiting for RGB-D topics"


class RGBDNode(Node):
    def __init__(self, state: State, color_topic: str, depth_topic: str):
        super().__init__("v5_direct_rgbd_server")
        self.state, self.bridge = state, CvBridge()
        self.create_subscription(Image, color_topic, self.color_cb, 10)
        self.create_subscription(Image, depth_topic, self.depth_cb, 10)

    def color_cb(self, msg: Image):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                raise RuntimeError("JPEG encode failed")
            with self.state.lock:
                self.state.jpeg = encoded.tobytes()
                self.state.color_stamp = time.time()
                self.state.color_encoding = msg.encoding
                self.state.frames += 1
                self.state.error = ""
        except Exception as exc:
            with self.state.lock: self.state.error = f"color: {exc}"

    def depth_cb(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if depth.dtype != np.uint16:
                depth = np.asarray(depth, dtype=np.uint16)
            with self.state.lock:
                self.state.depth = np.ascontiguousarray(depth).tobytes()
                view = np.clip((depth.astype(np.float32) - 150.0) / (1500.0 - 150.0) * 255.0, 0, 255).astype(np.uint8)
                view[depth == 0] = 0
                colorized = cv2.applyColorMap(255 - view, cv2.COLORMAP_TURBO)
                colorized[depth == 0] = 0
                ok, enc = cv2.imencode('.jpg', colorized, [cv2.IMWRITE_JPEG_QUALITY, 85])
                self.state.depth_jpeg = enc.tobytes() if ok else None
                self.state.depth_stamp = time.time()
                self.state.depth_encoding = msg.encoding
                self.state.depth_step = msg.step
        except Exception as exc:
            with self.state.lock: self.state.error = f"depth: {exc}"


def handler_for(state: State, stale_s: float):
    class Handler(BaseHTTPRequestHandler):
        def json(self, value, status=200):
            body = json.dumps(value, ensure_ascii=False).encode()
            self.send_response(status); self.send_header("Content-Type", "application/json"); self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)
        def do_GET(self):
            route = urlparse(self.path).path
            with state.lock:
                jpeg, depth, depth_jpeg, cs, ds, n = state.jpeg, state.depth, state.depth_jpeg, state.color_stamp, state.depth_stamp, state.frames
                ce, de, step, error = state.color_encoding, state.depth_encoding, state.depth_step, state.error
            now = time.time(); color_ok = jpeg is not None and now-cs <= stale_s; depth_ok = depth is not None and now-ds <= stale_s
            if route == "/health":
                self.json({"ok": color_ok and depth_ok, "camera_id":"wrist_direct_usb", "frame_count":n, "color_age_s":now-cs if cs else None, "depth_age_s":now-ds if ds else None, "image_size":[1280,720], "color_encoding":ce, "depth_encoding":de, "depth_bytes":len(depth) if depth else 0, "depth_step":step, "depth_unit":"millimetres (RealSense Z16 via ROS scale)", "metric_depth":depth_ok, "error":error}, 200 if color_ok and depth_ok else 503)
            elif route == "/frame.jpg":
                if not color_ok: self.send_error(503, error or "stale color frame"); return
                self.send_response(200); self.send_header("Content-Type","image/jpeg"); self.send_header("Content-Length",str(len(jpeg))); self.end_headers(); self.wfile.write(jpeg)
            elif route == "/depth.raw":
                if not depth_ok: self.send_error(503, error or "stale depth frame"); return
                self.send_response(200); self.send_header("Content-Type","application/octet-stream"); self.send_header("X-Depth-Width","1280"); self.send_header("X-Depth-Height","720"); self.send_header("X-Depth-Dtype","uint16"); self.send_header("Content-Length",str(len(depth))); self.end_headers(); self.wfile.write(depth)
            elif route == "/depth.jpg":
                if not depth_ok or depth_jpeg is None: self.send_error(503, error or "stale depth frame"); return
                self.send_response(200); self.send_header("Content-Type","image/jpeg"); self.send_header("Content-Length",str(len(depth_jpeg))); self.end_headers(); self.wfile.write(depth_jpeg)
            elif route == "/mjpeg":
                self.send_response(200); self.send_header("Content-Type","multipart/x-mixed-replace; boundary=frame"); self.end_headers(); last=-1
                try:
                    while True:
                        with state.lock: current, count = state.jpeg, state.frames
                        if current is not None and count != last:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "+str(len(current)).encode()+b"\r\n\r\n"+current+b"\r\n"); self.wfile.flush(); last=count
                        time.sleep(.03)
                except (BrokenPipeError, ConnectionResetError): pass
            elif route in ("/", "/index.html"):
                body = "<!doctype html><meta charset='utf-8'><title>V5 RGB-D</title><style>body{background:#111;color:#eee;text-align:center;font-family:sans-serif}.row{display:flex;gap:12px;justify-content:center;flex-wrap:wrap}.live{width:min(46vw,800px);height:auto;border:1px solid #555}p{word-break:break-all}</style><h2>V5 直连 Jetson · D435i · 1280×720 RGB-D</h2><div class='row'><div>彩色图<br><img class='live' src='/mjpeg'></div><div>深度图（伪彩色，150mm–1500mm）<br><img id='depth' class='live' src='/depth.jpg'></div></div><p id='s'>读取状态中…</p><script>const d=document.getElementById('depth');setInterval(()=>{d.src='/depth.jpg?t='+Date.now();fetch('/health').then(r=>r.json()).then(x=>s.textContent=JSON.stringify(x))},500)</script>"
                data=body.encode(); self.send_response(200); self.send_header("Content-Type","text/html; charset=utf-8"); self.send_header("Content-Length",str(len(data))); self.end_headers(); self.wfile.write(data)
            else: self.json({"service":"v5-direct-rgbd","health":"/health","color":"/frame.jpg","depth":"/depth.raw","stream":"/mjpeg"},404)
        def log_message(self, *_): pass
    return Handler


def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--color-topic",default="/camera/camera/color/image_raw"); ap.add_argument("--depth-topic",default="/camera/camera/aligned_depth_to_color/image_raw"); ap.add_argument("--port",type=int,default=8092); ap.add_argument("--stale-s",type=float,default=2.0); a=ap.parse_args()
    state=State(); rclpy.init(); node=RGBDNode(state,a.color_topic,a.depth_topic); threading.Thread(target=rclpy.spin,args=(node,),daemon=True).start(); print(f"V5 direct RGB-D http://0.0.0.0:{a.port}",flush=True); ThreadingHTTPServer(("0.0.0.0",a.port),handler_for(state,a.stale_s)).serve_forever()
if __name__ == "__main__": main()
