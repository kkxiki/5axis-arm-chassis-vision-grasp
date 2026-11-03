#!/usr/bin/env python3
"""Lightweight MJPEG server that streams the RealSense color image over HTTP.

Open http://<jetson-ip>:8090/ in a browser to view the live feed.
"""
import os
import threading
import time

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

BIND = "0.0.0.0"
PORT = int(os.environ.get("MJPEG_PORT", "8090"))
IMAGE_TOPIC = os.environ.get("MJPEG_TOPIC", "/camera/camera/color/image_raw")
QUALITY = int(os.environ.get("MJPEG_QUALITY", "70"))
MAX_WIDTH = int(os.environ.get("MJPEG_MAX_WIDTH", "640"))
ENCODE_FPS = float(os.environ.get("MJPEG_ENCODE_FPS", "15"))

_shared = {"jpeg": None, "lock": threading.Lock(), "fps": 0.0, "w": 0, "h": 0, "ok": False}


class CamSub(Node):
    def __init__(self):
        super().__init__("mjpeg_stream")
        self.last_encode = 0.0
        self.create_subscription(Image, IMAGE_TOPIC, self.on_img, 2)
        self.get_logger().info("subscribing to " + IMAGE_TOPIC)

    def on_img(self, msg):
        try:
            now = time.monotonic()
            if now - self.last_encode < 1.0 / max(1.0, ENCODE_FPS):
                return
            self.last_encode = now
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            shape = (msg.height, msg.width, int(msg.step) // msg.width)
            bgr = arr.reshape(shape)
            if bgr.shape[2] > 3:
                bgr = bgr[:, :, :3]
            h, w = bgr.shape[:2]
            if MAX_WIDTH and w > MAX_WIDTH:
                scale = MAX_WIDTH / float(w)
                bgr = cv2.resize(bgr, (int(w * scale), int(h * scale)),
                                 interpolation=cv2.INTER_AREA)
            ok, enc = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, QUALITY])
            if not ok:
                return
            with _shared["lock"]:
                _shared["jpeg"] = enc.tobytes()
                _shared["w"] = int(w)
                _shared["h"] = int(h)
                _shared["ok"] = True
        except Exception:
            pass


LOGIN_HTML = (
    "<!doctype html><html><head><meta charset='utf-8'><title>Jetson D435i Live</title>"
    "<style>body{background:#111;color:#eee;font-family:sans-serif;margin:0}"
    "img{width:100vw}</style></head><body><img src='/mjpeg'></body></html>"
)


def http_thread():
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                body = LOGIN_HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/mjpeg":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "close")
                self.end_headers()
                try:
                    while True:
                        with _shared["lock"]:
                            jpeg, w, h, ok = (
                                _shared["jpeg"], _shared["w"], _shared["h"], _shared["ok"])
                        if ok and jpeg:
                            head = ("--frame\r\nContent-Type: image/jpeg\r\n"
                                    "Content-Length: " + str(len(jpeg)) + "\r\n\r\n").encode()
                            self.wfile.write(head)
                            self.wfile.write(jpeg)
                            self.wfile.write(b"\r\n")
                        time.sleep(1.0 / max(1.0, ENCODE_FPS))
                except (BrokenPipeError, ConnectionResetError):
                    return
            elif self.path == "/status":
                with _shared["lock"]:
                    body = ("ok=%s fps=%.1f size=%dx%d" %
                            (_shared["ok"], _shared["fps"], _shared["w"], _shared["h"])).encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/plain")
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_response(404)
                self.end_headers()

    srv = ThreadingHTTPServer((BIND, PORT), H)
    print("MJPEG listening on http://%s:%d/" % (BIND, PORT), flush=True)
    srv.serve_forever()


def fps_thread():
    last_j = None
    last_t = time.time()
    cnt = 0
    while True:
        with _shared["lock"]:
            j = _shared["jpeg"]
        now = time.time()
        if j is not None and j is not last_j:
            cnt += 1
            last_j = j
        if now - last_t >= 2.0:
            with _shared["lock"]:
                _shared["fps"] = cnt / (now - last_t)
            cnt = 0
            last_t = now
        time.sleep(0.05)


def main():
    print("rclpy init...", flush=True)
    rclpy.init()
    node = CamSub()
    threading.Thread(target=http_thread, daemon=True).start()
    threading.Thread(target=fps_thread, daemon=True).start()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
