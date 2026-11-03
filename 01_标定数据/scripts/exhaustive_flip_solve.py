#!/usr/bin/env python3
import json,itertools,sys
import cv2,numpy as np
from scipy.spatial.transform import Rotation as R
from pathlib import Path

src,out=sys.argv[1],Path(sys.argv[2]);out.mkdir(parents=True,exist_ok=True)
d=json.load(open(src));s=d['samples'];n=len(s)
bg=[np.array(x['robot_pose']['base_T_gripper_m']) for x in s];ct0=[np.array(x['camera_T_target_m']) for x in s]
w=(d['board']['inner_corners_cols']-1)*d['board']['square_size_mm']/1000
h=(d['board']['inner_corners_rows']-1)*d['board']['square_size_mm']/1000
f=np.eye(4);f[:3,:3]=np.diag([-1.,-1.,1.]);f[:3,3]=[w,h,0];ct1=[x@f for x in ct0]
ra=np.zeros((n,n))
for i in range(n):
 for j in range(i+1,n): ra[i,j]=np.degrees(R.from_matrix(bg[i][:3,:3].T@bg[j][:3,:3]).magnitude())
best=None
for tail in itertools.product((0,1),repeat=n-1):
 bits=(0,)+tail; cost=[]
 for i in range(n):
  for j in range(i+1,n):
   a=(ct1[i] if bits[i] else ct0[i])[:3,:3];b=(ct1[j] if bits[j] else ct0[j])[:3,:3]
   ca=np.degrees(R.from_matrix(a@b.T).magnitude());cost.append(abs(ra[i,j]-ca))
 score=float(np.median(cost)+.1*np.mean(cost))
 if best is None or score<best[0]:best=(score,bits,cost)
_,bits,cost=best;ct=[ct1[i] if bits[i] else ct0[i] for i in range(n)]
def inv(t):
 o=np.eye(4);o[:3,:3]=t[:3,:3].T;o[:3,3]=-o[:3,:3]@t[:3,3];return o
def check(x):
 boards=[g@x@c for g,c in zip(bg,ct)];mt=np.mean([b[:3,3] for b in boards],0);mr=R.from_matrix([b[:3,:3] for b in boards]).mean()
 te=[np.linalg.norm(b[:3,3]-mt)*1000 for b in boards];re=[np.degrees((mr.inv()*R.from_matrix(b[:3,:3])).magnitude()) for b in boards]
 return np.sqrt(np.mean(np.square(te))),np.sqrt(np.mean(np.square(re)))
res=[]
for name,flag in [('tsai',0),('park',1),('horaud',2),('andreff',3),('daniilidis',4)]:
 try:
  rr,tt=cv2.calibrateHandEye(np.array([g[:3,:3] for g in bg]),np.array([g[:3,3] for g in bg]),np.array([c[:3,:3] for c in ct]),np.array([c[:3,3] for c in ct]),method=flag)
  x=np.eye(4);x[:3,:3]=rr;x[:3,3]=np.asarray(tt).reshape(3);te,re=check(x);res.append((te+2*re,name,x,te,re))
 except Exception:pass
_,name,x,te,re=min(res,key=lambda z:z[0]);q=R.from_matrix(x[:3,:3]).as_quat()
r={'method':name,'sample_count':n,'samples_used':[z['id'] for z in s],'flipped_ids':[s[i]['id'] for i,b in enumerate(bits) if b],
 'relative_rotation_delta_median_deg':float(np.median(cost)),'relative_rotation_delta_mean_deg':float(np.mean(cost)),
 'gripper_T_camera_m':x.tolist(),'translation_m':x[:3,3].tolist(),'quaternion_xyzw':q.tolist(),
 'translation_rms_mm':float(te),'rotation_rms_deg':float(re),'quality_passed':bool(te<10 and re<2)}
(out/'eye_in_hand_result.json').write_text(json.dumps(r,indent=2),encoding='utf-8');print(json.dumps(r,indent=2))
