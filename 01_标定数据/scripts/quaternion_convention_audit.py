#!/usr/bin/env python3
import itertools,json,sys
import numpy as np
from scipy.spatial.transform import Rotation as R
d=json.load(open(sys.argv[1]));s=d['samples'];n=len(s);c0=[np.array(x['camera_T_target_m']) for x in s]
w=(d['board']['inner_corners_cols']-1)*d['board']['square_size_mm']/1000;h=(d['board']['inner_corners_rows']-1)*d['board']['square_size_mm']/1000
f=np.eye(4);f[:3,:3]=np.diag([-1.,-1.,1.]);f[:3,3]=[w,h,0];c1=[x@f for x in c0];raw=np.array([x['robot_pose']['raw'][3:7] for x in s]);rng=np.random.default_rng(3)
results=[]
for perm in itertools.permutations(range(4)):
 for signs3 in itertools.product((-1,1),repeat=3):
  signs=np.array([*signs3,1]);q=raw[:,perm]*signs
  try:rr=R.from_quat(q).as_matrix()
  except:continue
  pairs=[]
  for i in range(n):
   for j in range(i+1,n):
    ra=np.degrees(R.from_matrix(rr[i].T@rr[j]).magnitude());cost=np.zeros((2,2))
    for a in (0,1):
     for b in (0,1):
      aa=(c1[i] if a else c0[i])[:3,:3];bb=(c1[j] if b else c0[j])[:3,:3];ca=np.degrees(R.from_matrix(aa@bb.T).magnitude());cost[a,b]=abs(ra-ca)
    pairs.append((i,j,cost))
  def score(z):return sum(np.minimum(c[z[i],z[j]],30) for i,j,c in pairs)
  best=None
  for run in range(6):
   z=np.r_[0,rng.integers(0,2,n-1)]
   for _ in range(10):
    changed=False
    for i in range(1,n):
     old=z[i];z[i]=0;a=score(z);z[i]=1;b=score(z);z[i]=int(b<a);changed|=z[i]!=old
    if not changed:break
   sc=score(z)
   if best is None or sc<best[0]:best=(sc,z.copy())
  z=best[1];deltas=[c[z[i],z[j]] for i,j,c in pairs]
  results.append((float(np.median(deltas)),float(np.percentile(deltas,90)),perm,signs.tolist(),z.tolist()))
for x in sorted(results)[:20]:print(x)
