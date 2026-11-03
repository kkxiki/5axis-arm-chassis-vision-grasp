#!/usr/bin/env python3
import argparse,json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

def mat(v):
 t=np.eye(4); t[:3,:3]=Rotation.from_rotvec(v[3:]).as_matrix(); t[:3,3]=v[:3]; return t
def vec(t): return np.r_[t[:3,3],Rotation.from_matrix(t[:3,:3]).as_rotvec()]
def inv(t):
 o=np.eye(4);o[:3,:3]=t[:3,:3].T;o[:3,3]=-o[:3,:3]@t[:3,3];return o
def main():
 p=argparse.ArgumentParser();p.add_argument('--samples',required=True);p.add_argument('--initial',required=True);p.add_argument('--output',required=True);a=p.parse_args()
 man=json.load(open(a.samples)); ini=json.load(open(a.initial)); used=set(ini['samples_used']); flips=set(ini['checkerboard_flipped_ids'])
 ss=[s for s in man['samples'] if s['id'] in used]; bg=[np.array(s['robot_pose']['base_T_gripper_m']) for s in ss]
 w=(man['board']['inner_corners_cols']-1)*man['board']['square_size_mm']/1000;h=(man['board']['inner_corners_rows']-1)*man['board']['square_size_mm']/1000
 f=np.eye(4);f[:3,:3]=np.diag([-1.,-1.,1.]);f[:3,3]=[w,h,0]
 ct=[np.array(s['camera_T_target_m'])@(f if s['id'] in flips else np.eye(4)) for s in ss]
 x0=np.array(ini['gripper_T_camera_m']); boards=[g@x0@c for g,c in zip(bg,ct)]
 b0=np.eye(4);b0[:3,3]=np.median([b[:3,3] for b in boards],axis=0);b0[:3,:3]=Rotation.from_matrix([b[:3,:3] for b in boards]).mean().as_matrix()
 def fun(z):
  x,b=mat(z[:6]),mat(z[6:]); out=[]
  for g,c in zip(bg,ct):
   d=inv(b)@g@x@c; out.extend(d[:3,3]/0.005);out.extend(Rotation.from_matrix(d[:3,:3]).as_rotvec()/np.radians(1.0))
  return out
 opt=least_squares(fun,np.r_[vec(x0),vec(b0)],loss='soft_l1',f_scale=2,max_nfev=4000)
 x,b=mat(opt.x[:6]),mat(opt.x[6:]);te=[];re=[]
 for g,c in zip(bg,ct):
  d=inv(b)@g@x@c;te.append(np.linalg.norm(d[:3,3])*1000);re.append(np.degrees(Rotation.from_matrix(d[:3,:3]).magnitude()))
 result=dict(ini);result.update({'method':'joint_nonlinear_soft_l1','gripper_T_camera_m':x.tolist(),'translation_m':x[:3,3].tolist(),'quaternion_xyzw':Rotation.from_matrix(x[:3,:3]).as_quat().tolist(),'translation_rms_mm':float(np.sqrt(np.mean(np.square(te)))),'translation_max_mm':float(max(te)),'rotation_rms_deg':float(np.sqrt(np.mean(np.square(re)))),'rotation_max_deg':float(max(re)),'optimization_success':bool(opt.success),'quality_passed':bool(np.sqrt(np.mean(np.square(te)))<10 and np.sqrt(np.mean(np.square(re)))<2)})
 Path(a.output).write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
