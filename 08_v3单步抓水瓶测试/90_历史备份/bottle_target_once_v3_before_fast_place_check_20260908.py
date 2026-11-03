#!/usr/bin/env python3
"""YOLOv8 COCO bottle (class 39) detector with fixed-pose hand-eye localization."""
import json
import os
import time
from pathlib import Path
import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray

CLASS_ID = "39"
MIN_SCORE = 0.30
CALIBRATION = "/home/nvidia/calibration/标定数据/v3_重新采集_20260907/wrist_d435i_to_tcp_corrected.yaml"
PHOTO_POSE = "/home/nvidia/calibration/poses/grasp_photo_pose_v3.json"

def quat_to_rot(q):
    x, y, z, w = [float(v) for v in q]
    n = x*x + y*y + z*z + w*w
    if n < 1e-12: return np.eye(3)
    s = 2.0 / n
    return np.array([[1-s*(y*y+z*z), s*(x*y-z*w), s*(x*z+y*w)],
                     [s*(x*y+z*w), 1-s*(x*x+z*z), s*(y*z-x*w)],
                     [s*(x*z-y*w), s*(y*z+x*w), 1-s*(x*x+y*y)]], dtype=float)

def bbox_center(bbox):
    c = bbox.center
    return (float(c.position.x), float(c.position.y)) if hasattr(c, "position") else (float(c.x), float(c.y))

class Detector(Node):
    def __init__(self):
        super().__init__("yolov8_bottle_target_v3_once")
        self.bridge, self.color, self.depth, self.info = CvBridge(), None, None, None
        self.candidates, self.detection_time = [], 0.0
        # BOTTLE_CHECK_MODE=1 => occupancy/count probe only: no full hand-eye
        # localization.  The S6 place-area check uses the number of YOLO
        # bottle boxes inside the configured ROI.
        self.check_mode = os.environ.get("BOTTLE_CHECK_MODE") == "1"
        # Optional normalized ROI "x0 y0 x1 y1" (0..1): only bottle boxes whose
        # centre falls inside are considered (used for the place-area check).
        self.roi_norm = None
        roi_raw = os.environ.get("BOTTLE_ROI_NORM")
        if roi_raw:
            try:
                a = [float(x) for x in roi_raw.replace(",", " ").split()]
                if (len(a) == 4 and 0.0 <= a[0] <= a[2] <= 1.0
                        and 0.0 <= a[1] <= a[3] <= 1.0):
                    self.roi_norm = a
                    self.get_logger().info(f"ROI_NORM enabled: {a}")
            except ValueError:
                pass
        self.t_base_camera = self.load_transform()
        self.create_subscription(Image, "/camera/camera/color/image_raw", self.color_cb, 10)
        self.create_subscription(Image, "/camera/camera/aligned_depth_to_color/image_raw", self.depth_cb, 10)
        self.create_subscription(CameraInfo, "/camera/camera/color/camera_info", self.info_cb, 10)
        self.create_subscription(Detection2DArray, "/yolov8/detections", self.detection_cb, 10)

    def load_transform(self):
        tcp = json.loads(Path(PHOTO_POSE).read_text(encoding="utf-8"))["robottarget"]
        cal = yaml.safe_load(Path(CALIBRATION).read_text(encoding="utf-8"))
        r_bt, t_bt = quat_to_rot(tcp[3:7]), np.asarray(tcp[:3], float)
        r_tc, t_tc = quat_to_rot(cal["quaternion_xyzw"]), np.asarray(cal["translation_m"], float)
        return r_bt @ r_tc, t_bt + r_bt @ t_tc

    def color_cb(self, msg): self.color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    def depth_cb(self, msg):
        a = self.bridge.imgmsg_to_cv2(msg, "passthrough").astype(np.float32)
        self.depth = a * (0.001 if msg.encoding == "16UC1" else 1.0)
    def info_cb(self, msg): self.info = msg
    def detection_cb(self, msg):
        candidates = []
        for d in msg.detections:
            for r in d.results:
                h = r.hypothesis if hasattr(r, "hypothesis") else r
                if str(h.class_id) == CLASS_ID and float(h.score) >= MIN_SCORE:
                    candidates.append((float(h.score), d))
        if candidates:
            self.candidates = candidates
            self.detection_time = time.monotonic()

    def detect(self):
        if self.color is None or self.depth is None or self.info is None or not self.candidates:
            return None
        if time.monotonic() - self.detection_time > 1.0: return None
        h, w = self.depth.shape[:2]
        # pick the highest-score bottle whose bbox centre is inside the ROI
        # (if one was set); without ROI every detection is allowed.
        best = None
        for score, det in self.candidates:
            ncx, ncy = bbox_center(det.bbox)
            if ncx > 1.5: ncx /= w
            if ncy > 1.5: ncy /= h
            if self.roi_norm is not None and not (self.roi_norm[0] <= ncx <= self.roi_norm[2]
                                                  and self.roi_norm[1] <= ncy <= self.roi_norm[3]):
                continue
            if best is None or score > best[0]:
                best = (score, det, ncx * w, ncy * h)
        if best is None:
            return None
        score, det, cx, cy = best
        bw, bh = float(det.bbox.size_x), float(det.bbox.size_y)
        if 0 < bw <= 1.5 and 0 < bh <= 1.5: bw, bh = bw*w, bh*h
        # Depth is sampled on the opaque red label, while the target pixel is
        # the YOLO bottle-box center. This explicitly compensates the label
        # center -> bottle center image offset instead of grasping at the label.
        depth_u = int(round(np.clip(cx, 8, w-9)))
        depth_v = int(round(np.clip(cy + 0.30*bh, 8, h-9)))
        target_u, target_v = float(cx), float(cy)
        patch = self.depth[depth_v-7:depth_v+8, depth_u-7:depth_u+8]
        valid = patch[(patch >= 0.15) & (patch <= 1.50)]
        if valid.size < 30: return None
        z = float(np.median(valid))
        fx, fy, px, py = self.info.k[0], self.info.k[4], self.info.k[2], self.info.k[5]
        p_cam = np.array([(target_u-px)*z/fx, (target_v-py)*z/fy, z], float)
        r, t = self.t_base_camera
        p_base = r @ p_cam + t
        x, y = int(round(cx-bw/2)), int(round(cy-bh/2))
        debug = self.color.copy()
        if self.roi_norm is not None:
            rx0, ry0 = int(round(self.roi_norm[0]*w)), int(round(self.roi_norm[1]*h))
            rx1, ry1 = int(round(self.roi_norm[2]*w)), int(round(self.roi_norm[3]*h))
            cv2.rectangle(debug, (rx0, ry0), (rx1, ry1), (255, 0, 255), 2)
            cv2.putText(debug, "ROI", (rx0+5, ry0+22), cv2.FONT_HERSHEY_SIMPLEX, .7, (255, 0, 255), 2)
        cv2.rectangle(debug, (x,y), (x+int(round(bw)), y+int(round(bh))), (0,255,0), 3)
        cv2.circle(debug, (depth_u,depth_v), 7, (255,0,0), -1)
        cv2.circle(debug, (int(round(target_u)),int(round(target_v))), 7, (0,255,255), -1)
        cv2.putText(debug, f"bottle39 {score:.2f} depth={z:.3f}m", (max(5,x), max(25,y-10)), cv2.FONT_HERSHEY_SIMPLEX, .8, (0,255,0), 2)
        return {"pixel":[int(round(target_u)),int(round(target_v))],
                "bbox_center_pixel":[target_u,target_v],
                "depth_sample_pixel":[depth_u,depth_v],
                "pixel_offset_applied":[target_u-depth_u,target_v-depth_v],
                "bbox":[x,y,int(round(bw)),int(round(bh))], "score":score,
                "depth_m":z, "base_xyz_m":p_base.tolist(), "debug":debug}

    def count_in_roi(self):
        """Count current bottle boxes whose centres are inside the ROI."""
        if not self.candidates or time.monotonic() - self.detection_time > 1.0:
            return 0
        h, w = self.depth.shape[:2] if self.depth is not None else (480, 640)
        count = 0
        for score, det in self.candidates:
            ncx, ncy = bbox_center(det.bbox)
            if ncx > 1.5: ncx /= w
            if ncy > 1.5: ncy /= h
            if self.roi_norm is None or (self.roi_norm[0] <= ncx <= self.roi_norm[2]
                                         and self.roi_norm[1] <= ncy <= self.roi_norm[3]):
                count += 1
        return count

def main():
    rclpy.init(); node = Detector()
    try:
        if node.check_mode:
            CHECK_MAX_S, CHECK_HITS = 9.0, 3  # require three fresh frames
            frame_counts = []
            deadline = time.time() + CHECK_MAX_S
            last_cap = 0.0
            while time.time() < deadline and len(frame_counts) < CHECK_HITS:
                rclpy.spin_once(node, timeout_sec=.1)
                if time.time() - last_cap < .25: continue
                count = node.count_in_roi()
                if count > 0:
                    frame_counts.append(count)
                last_cap = time.time()
            bottle_count = max(frame_counts) if frame_counts else 0
            out = {"detected": bool(bottle_count > 0), "bottle_count": bottle_count,
                   "method": "quick_roi_bottle_count_check", "sample_count": len(frame_counts),
                   "frame_counts": frame_counts,
                   "reason": "occupied" if bottle_count > 0 else "no bottle inside ROI"}
            Path("/tmp/v3_bottle_target.json").write_text(json.dumps(out, ensure_ascii=False, indent=2) + "\n")
            print(json.dumps(out, ensure_ascii=False, indent=2))
            return
        samples=[]; last=None; deadline=time.time()+20; last_cap=0
        while time.time()<deadline and len(samples)<15:
            rclpy.spin_once(node, timeout_sec=.1)
            if time.time()-last_cap < .25: continue
            result=node.detect()
            if result is None: continue
            samples.append(result["base_xyz_m"]); last=result; last_cap=time.time()
        if len(samples)<11 or last is None: raise RuntimeError(f"insufficient YOLO bottle detections: {len(samples)}")
        a=np.asarray(samples,float); med=np.median(a,axis=0); dev=np.linalg.norm(a-med,axis=1); inside=a[dev<=.008]
        if len(inside)<9: raise RuntimeError(f"unstable bottle 3D target: {len(inside)}/{len(samples)} inliers")
        med=np.median(inside,axis=0); maxdev=float(np.max(np.linalg.norm(inside-med,axis=1)))
        out={"detected":True,"method":"yolov8_bottle_class39_fixed_photo_handeye_v3","sample_count":len(samples),"inlier_count":len(inside),"base_xyz_m":med.tolist(),"max_deviation_m":maxdev,"last_frame":{k:v for k,v in last.items() if k!="debug"},"calibration":CALIBRATION,"photo_pose":PHOTO_POSE}
        Path("/tmp/v3_bottle_target.json").write_text(json.dumps(out,ensure_ascii=False,indent=2)+"\n")
        cv2.imwrite("/tmp/v3_bottle_label_target.jpg", last["debug"]); print(json.dumps(out,ensure_ascii=False,indent=2))
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()
