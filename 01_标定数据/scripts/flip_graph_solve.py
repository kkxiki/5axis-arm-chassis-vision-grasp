#!/usr/bin/env python3
import json,sys
from pathlib import Path
import cv2,numpy as np
from scipy.spatial.transform import Rotation as R

d=json.load(open(sys.argv[1])); out=Path(sys.argv[2]);out.mkdir(parents=True,exist_ok=True); s=d['samples'];n=len(s)
bg=[np.array(x['robot_pose']['base_T_gripper_m']) for x in s]; c0=[np.array(x['camera_T_target_m']) for x in s]
w=(d['board']['inner_corners_cols']-1)*d['board']['square_size_mm']/1000; h=(d['board']['inner_corners_rows']-1)*d['board']['square_size_mm']/1000
f=np.eye(4);f[:3,:3]=np.diag([-1.,-1.,1.]);f[:3,3]=[w,h,0]; c1=[x@f for x in c0]
pair=[]
for i in range(n):
 for j in range(i+1,n):
  ra=np.degrees(R.from_matrix(bg[i][:3,:3].T@bg[j][:3,:3]).magnitude())
  costs=np.empty((2,2))
  for a in (0,1):
   for b in (0,1):
    aa=(c1[i] if a else c0[i])[:3,:3];bb=(c1[j] if b else c0[j])[:3,:3]
    ca=np.degrees(R.from_matrix(aa@bb.T).magnitude()); delta=abs(ra-ca)
    costs[a,b]=delta if delta<10 else 10+np.sqrt(delta-10)
  pair.append((i,j,costs))
def score(bits): return sum(c[bits[i],bits[j]] for i,j,c in pair)
rng=np.random.default_rng(20260902);best=None
for run in range(5000):
 bits=np.r_[0,rng.integers(0,2,n-1)]
 changed=True
 while changed:
  changed=False
  for i in rng.permutation(np.arange(1,n)):
   old=score(bits);bits[i]^=1;new=score(bits)
   if new+1e-9<old:changed=True
   else:bits[i]^=1
 z=score(bits)
 if best is None or z<best[0]:best=(z,bits.copy())
bits=best[1];ct=[c1[i] if bits[i] else c0[i] for i in range(n)]
def validate(x):
 boards=[g@x@c for g,c in zip(bg,ct)];mt=np.mean([b[:3,3] for b in boards],0);mr=R.from_matrix([b[:3,:3] for b in boards]).mean();te=[];re=[]
 for b in boards:te.append(np.linalg.norm(b[:3,3]-mt)*1000);re.append(np.degrees((mr.inv()*R.from_matrix(b[:3,:3])).magnitude()))
 return np.sqrt(np.mean(np.square(te))),np.sqrt(np.mean(np.square(re))),te,re
res=[]
for name,flag in [('tsai',0),('park',1),('horaud',2),('andreff',3),('daniilidis',4)]:
 try:
  rr,tt=cv2.calibrateHandEye(np.array([g[:3,:3] for g in bg]),np.array([g[:3,3] for g in bg]),np.array([c[:3,:3] for c in ct]),np.array([c[:3,3] for c in ct]),method=flag);x=np.eye(4);x[:3,:3]=rr;x[:3,3]=np.asarray(tt).reshape(3);te,re,_,_=validate(x);res.append((te+2*re,name,x,te,re))
 except Exception:pass
_,name,x,te,re=min(res); deltas=[]
for i,j,c in pair:
 # report the true unbounded angle delta for selected labels
 ra=np.degrees(R.from_matrix(bg[i][:3,:3].T@bg[j][:3,:3]).magnitude());aa=ct[i][:3,:3];bb=ct[j][:3,:3];ca=np.degrees(R.from_matrix(aa@bb.T).magnitude());deltas.append(abs(ra-ca))
result={'method':name,'sample_count':n,'flipped_ids':[s[i]['id'] for i,b in enumerate(bits) if b],'relative_angle_delta_median_deg':float(np.median(deltas)),'relative_angle_delta_p90_deg':float(np.percentile(deltas,90)),'relative_angle_delta_max_deg':float(max(deltas)),'translation_m':x[:3,3].tolist(),'quaternion_xyzw':R.from_matrix(x[:3,:3]).as_quat().tolist(),'gripper_T_camera_m':x.tolist(),'translation_rms_mm':float(te),'rotation_rms_deg':float(re),'quality_passed':bool(te<10 and re<2)}
(out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result,indent=2))
