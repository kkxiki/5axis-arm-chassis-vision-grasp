#!/usr/bin/env python3
"""Local ROI picker: user drags a rectangle over the place-photo view and saves
the pixel box. Output JSON (pixel + normalized ROI for a 1280x720 image) is
written next to the image inside the v3 test folder."""
import json
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
V3_DIR = next(p for p in ROOT.iterdir() if p.is_dir() and p.name.startswith("08_v3"))
IMG_PATH = V3_DIR / "place_photo_occupy_view.jpg"
OUT_PATH = V3_DIR / "place_check_roi.json"
W, H = 1280, 720

PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>ROI picker</title>
<style>
 body{background:#111;color:#eee;font-family:system-ui;margin:16px}
 .cvbox{position:relative;display:inline-block;width:min(1100px, 92vw);border:1px solid #444}
 .cvbox canvas{width:100%;display:block}
 #base{height:auto}
 #cv{position:absolute;left:0;top:0;height:100%;cursor:crosshair}
 #info{margin:10px 0;font-size:14px;min-height:1.2em;color:#ff0}
 #tip{font-size:13px;color:#aaa;margin-bottom:6px}
 button{margin-right:8px;padding:8px 18px;font-size:15px;cursor:pointer}
 #msg{color:#7f7;font-size:14px}
</style>
</head>
<body>
<h3>Draw the region to check: click corner 1, then click the opposite corner (drag also works), then Save</h3>
<div id="tip">corner mode / drag mode both supported</div>
<div id="info">-</div>
<div class="cvbox">
  <canvas id="base" width="1280" height="720"></canvas>
  <canvas id="cv" width="1280" height="720"></canvas>
</div>
<br>
<button id="save">Save ROI</button>
<button id="clr">Clear</button>
<span id="msg"></span>
<script>
var img=document.getElementById('base');
var cv=document.getElementById('cv');
var bctx=img.getContext('2d');
var cctx=cv.getContext('2d');
var c1=null;      // first click corner
var rect=null;    // final box {x0,y0,x1,y1}
var sx=0,sy=0,ex=0,ey=0,drag=false,moved=false;
var photo=new Image();
photo.onload=function(){bctx.drawImage(photo,0,0);};
photo.src='/image';
function evpos(e){
 var r=cv.getBoundingClientRect();
 return{x:(e.clientX-r.left)*W/r.width, y:(e.clientY-r.top)*H/r.height};
}
function norm(a,b){return{x0:Math.round(Math.min(a.x,b.x)),y0:Math.round(Math.min(a.y,b.y)),
 x1:Math.round(Math.max(a.x,b.x)),y1:Math.round(Math.max(a.y,b.y))};}
function paintBox(b){
 cctx.strokeStyle='#ff00ff';cctx.lineWidth=4;
 cctx.strokeRect(b.x0+2,b.y0+2,b.x1-b.x0-4,b.y1-b.y0-4);
 cctx.fillStyle='rgba(255,0,255,0.18)';
 cctx.fillRect(b.x0+2,b.y0+2,b.x1-b.x0-4,b.y1-b.y0-4);
 cctx.fillStyle='#ff00ff';cctx.font='24px system-ui';
 cctx.fillText('ROI ('+b.x0+','+b.y0+') -> ('+b.x1+','+b.y1+')', b.x0+6, Math.max(26, b.y0-10));
}
function dot(p){
 cctx.fillStyle='#ff0';cctx.beginPath();cctx.arc(p.x,p.y,7,0,6.2832);cctx.fill();
}
function redraw(){
 cctx.clearRect(0,0,W,H);
 if(drag){paintBox(norm({x:sx,y:sy},{x:ex,y:ey}));}
 else if(rect){paintBox(rect);}
 else if(c1){dot(c1);}
}
function showInfo(t){document.getElementById('info').textContent=t;}
function down(e){e.preventDefault();var p=evpos(e);sx=p.x;sy=p.y;ex=p.x;ey=p.y;
 drag=true;moved=false;redraw();}
function move(e){if(!drag)return;e.preventDefault();var p=evpos(e);ex=p.x;ey=p.y;
 if(Math.abs(ex-sx)+Math.abs(ey-sy)>6){moved=true;}
 redraw();
 if(moved){var n=norm({x:sx,y:sy},{x:ex,y:ey});showInfo('drag: '+n.x0+','+n.y0+' -> '+n.x1+','+n.y1);}}
function up(e){
 if(!drag)return;
 drag=false;
 if(!moved){ // plain click
   if(c1===null){c1={x:sx,y:sy};rect=null;
     showInfo('corner 1 set at ('+Math.round(sx)+','+Math.round(sy)+') - click the opposite corner');
   }else{rect=norm(c1,{x:sx,y:sy});c1=null;
     showInfo('pixel box: x0='+rect.x0+' y0='+rect.y0+' x1='+rect.x1+' y1='+rect.y1);
   }
 }else{rect=norm({x:sx,y:sy},{x:ex,y:ey});c1=null;
   showInfo('pixel box: x0='+rect.x0+' y0='+rect.y0+' x1='+rect.x1+' y1='+rect.y1);
 }
 redraw();
}
if(window.PointerEvent){
 cv.addEventListener('pointerdown',down);
 cv.addEventListener('pointermove',move);
 window.addEventListener('pointerup',up);
 cv.addEventListener('pointercancel',up);
}else{
 cv.addEventListener('mousedown',down);
 cv.addEventListener('mousemove',move);
 window.addEventListener('mouseup',up);
}
document.getElementById('clr').onclick=function(){c1=null;rect=null;drag=false;redraw();
 document.getElementById('info').textContent='-';document.getElementById('msg').textContent='';};
document.getElementById('save').onclick=function(){
 if(!rect){alert('draw a rectangle first');return;}
 var b=rect;
 fetch('/save',{method:'POST',headers:{'Content-Type':'application/json'},
   body:JSON.stringify({x0:b.x0,y0:b.y0,x1:b.x1,y1:b.y1})})
  .then(function(r){return r.json();})
  .then(function(j){ document.getElementById('msg').textContent='saved roi_norm: '+j.roi_norm.map(function(v){return v.toFixed(3);}).join('  '); })
  .catch(function(e){ alert('save failed: '+e); });
};
</script>
</body>
</html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="text/html; charset=utf-8"):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/image":
            self._send(200, IMG_PATH.read_bytes(), "image/jpeg")
        else:
            self._send(200, PAGE.encode("utf-8"))

    def do_POST(self):
        if self.path != "/save":
            self._send(404, b"not found")
            return
        n = int(self.headers.get("Content-Length", 0))
        try:
            b = json.loads(self.rfile.read(n))
            x0, y0 = int(b["x0"]), int(b["y0"])
            x1, y1 = int(b["x1"]), int(b["y1"])
            x0, x1 = sorted((max(0, x0), min(W, x1)))
            y0, y1 = sorted((max(0, y0), min(H, y1)))
            if x1 - x0 < 20 or y1 - y0 < 20:
                self._send(400, json.dumps({"error": "box too small"}).encode())
                return
            out = {"pixel": [x0, y0, x1, y1], "image_w": W, "image_h": H,
                   "roi_norm": [round(x0 / W, 4), round(y0 / H, 4),
                                round(x1 / W, 4), round(y1 / H, 4)]}
            OUT_PATH.write_text(json.dumps(out, ensure_ascii=False, indent=2) + "\n",
                                encoding="utf-8")
            self._send(200, json.dumps(out).encode(), "application/json")
        except Exception as exc:
            self._send(400, json.dumps({"error": str(exc)}).encode())


def main():
    if not IMG_PATH.exists():
        print(f"image missing: {IMG_PATH}")
        return 1
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8901
    print(f"ROI picker: http://127.0.0.1:{port}")
    ThreadingHTTPServer(("127.0.0.1", port), Handler).serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
