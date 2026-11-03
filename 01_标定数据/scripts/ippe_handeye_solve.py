#!/usr/bin/env python3
import json,sys
from pathlib import Path
import cv2,numpy as np
from scipy.spatial.transform import Rotation as R

src=Path(sys.argv[1]);out=Path(sys.argv[2]);out.mkdir(parents=True,exist_ok=True);d=json.load(open(src));s=d['samples'];n=len(s)
cols=d['board']['inner_corners_cols'];rows=d['board']['inner_corners_rows'];sq=d['board']['square_size_mm']/1000
obj=np.zeros((cols*rows,3),np.float32);obj[:,:2]=np.mgrid[0:cols,0:rows].T.reshape(-1,2);obj*=sq
K=np.array(d['camera']['k']).reshape(3,3);D=np.array(d['camera']['d']);bg=[np.array(x['robot_pose']['base_T_gripper_m']) for x in s];cand=[]
for sample in s:
 im=cv2.imread(str(src.parent/sample['image']));gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
 ok,corners=cv2.findChessboardCornersSB(gray,(cols,rows),flags=cv2.CALIB_CB_NORMALIZE_IMAGE|cv2.CALIB_CB_EXHAUSTIVE|cv2.CALIB_CB_ACCURACY)
 if not ok:
  ok,corners=cv2.findChessboardCorners(gray,(cols,rows),cv2.CALIB_CB_ADAPTIVE_THRESH|cv2.CALIB_CB_NORMALIZE_IMAGE)
  if ok: corners=cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,40,.001))
 if not ok:raise RuntimeError('board missing '+str(sample['id']))
 cs=[]
 for pts in (corners,corners[::-1].copy()):
  ret,rvecs,tvecs,errs=cv2.solvePnPGeneric(obj,pts,K,D,flags=cv2.SOLVEPNP_IPPE)
  for rv,tv in zip(rvecs,tvecs):
   if float(tv[2])<=0:continue
   t=np.eye(4);t[:3,:3]=cv2.Rodrigues(rv)[0];t[:3,3]=tv.reshape(3);cs.append(t)
 cand.append(cs)
print('candidate_counts',[len(x) for x in cand])
pair=[]
for i in range(n):
 for j in range(i+1,n):
  ra=np.degrees(R.from_matrix(bg[i][:3,:3].T@bg[j][:3,:3]).magnitude()); cost=np.zeros((len(cand[i]),len(cand[j])))
  for a in range(len(cand[i])):
   for b in range(len(cand[j])):
    ca=np.degrees(R.from_matrix(cand[i][a][:3,:3]@cand[j][b][:3,:3].T).magnitude());delta=abs(ra-ca);cost[a,b]=delta if delta<10 else 10+np.sqrt(delta-10)
  pair.append((i,j,cost))
def score(z):return sum(c[z[i],z[j]] for i,j,c in pair)
rng=np.random.default_rng(9);best=None
for run in range(250):
 z=np.array([rng.integers(len(x)) for x in cand]);changed=True
 while changed:
  changed=False
  for i in rng.permutation(n):
   old=z[i];vals=[]
   for v in range(len(cand[i])):z[i]=v;vals.append(score(z))
   z[i]=int(np.argmin(vals));changed|=z[i]!=old
 sc=score(z)
 if best is None or sc<best[0]:best=(sc,z.copy())
z=best[1];ct=[cand[i][z[i]] for i in range(n)]
def val(x):
 boards=[g@x@c for g,c in zip(bg,ct)];mt=np.mean([b[:3,3] for b in boards],0);mr=R.from_matrix([b[:3,:3] for b in boards]).mean();te=[];re=[]
 for b in boards:te.append(np.linalg.norm(b[:3,3]-mt)*1000);re.append(np.degrees((mr.inv()*R.from_matrix(b[:3,:3])).magnitude()))
 return np.sqrt(np.mean(np.square(te))),np.sqrt(np.mean(np.square(re)))
res=[]
for name,flag in [('tsai',0),('park',1),('horaud',2),('andreff',3),('daniilidis',4)]:
 try:
  rr,tt=cv2.calibrateHandEye(np.array([g[:3,:3] for g in bg]),np.array([g[:3,3] for g in bg]),np.array([c[:3,:3] for c in ct]),np.array([c[:3,3] for c in ct]),method=flag);x=np.eye(4);x[:3,:3]=rr;x[:3,3]=np.asarray(tt).reshape(3);te,re=val(x);res.append((te+2*re,name,x,te,re))
 except:pass
_,name,x,te,re=min(res);deltas=[]
for i,j,c in pair:
 ra=np.degrees(R.from_matrix(bg[i][:3,:3].T@bg[j][:3,:3]).magnitude());ca=np.degrees(R.from_matrix(ct[i][:3,:3]@ct[j][:3,:3].T).magnitude());deltas.append(abs(ra-ca))
r={'method':name,'pnp':'IPPE globally selected','sample_count':n,'candidate_index':z.tolist(),'relative_angle_delta_median_deg':float(np.median(deltas)),'relative_angle_delta_p90_deg':float(np.percentile(deltas,90)),'translation_rms_mm':float(te),'rotation_rms_deg':float(re),'translation_m':x[:3,3].tolist(),'quaternion_xyzw':R.from_matrix(x[:3,:3]).as_quat().tolist(),'gripper_T_camera_m':x.tolist(),'quality_passed':bool(te<10 and re<2)}
(out/'result.json').write_text(json.dumps(r,indent=2),encoding='utf-8');print(json.dumps(r,indent=2))
