#!/usr/bin/env python3
"""Robust eye-in-hand solve with checkerboard 180-degree ambiguity handling."""
import argparse, json
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

METHODS = [("tsai", cv2.CALIB_HAND_EYE_TSAI), ("park", cv2.CALIB_HAND_EYE_PARK),
           ("horaud", cv2.CALIB_HAND_EYE_HORAUD), ("daniilidis", cv2.CALIB_HAND_EYE_DANIILIDIS)]

def inv(t):
    o=np.eye(4); o[:3,:3]=t[:3,:3].T; o[:3,3]=-o[:3,:3]@t[:3,3]; return o

def mean_pose(ts):
    o=np.eye(4); o[:3,3]=np.mean([t[:3,3] for t in ts],axis=0)
    o[:3,:3]=Rotation.from_matrix([t[:3,:3] for t in ts]).mean().as_matrix(); return o

def errors(ts, mean):
    te=[]; re=[]
    for t in ts:
        d=inv(mean)@t; te.append(np.linalg.norm(d[:3,3])*1000)
        re.append(np.degrees(Rotation.from_matrix(d[:3,:3]).magnitude()))
    return np.array(te),np.array(re)

def handeye(bg, ct, flag):
    r,t=cv2.calibrateHandEye(np.array([x[:3,:3] for x in bg]),np.array([x[:3,3] for x in bg]),
                             np.array([x[:3,:3] for x in ct]),np.array([x[:3,3] for x in ct]),method=flag)
    x=np.eye(4); x[:3,:3]=r; x[:3,3]=np.asarray(t).reshape(3); return x

def score(bg,ct,x):
    boards=[a@x@b for a,b in zip(bg,ct)]; m=mean_pose(boards); te,re=errors(boards,m)
    return float(np.sqrt(np.mean(te**2))+2*np.sqrt(np.mean(re**2))),te,re,m

def main():
    p=argparse.ArgumentParser(); p.add_argument('--samples',required=True); p.add_argument('--out-dir',required=True)
    p.add_argument('--pose-convention',choices=('stored','wxyz','stored_inverse','wxyz_inverse'),default='stored')
    a=p.parse_args(); manifest=json.load(open(a.samples,encoding='utf-8'))
    raw=[s for s in manifest['samples'] if s['reprojection_rms_px'] <= 1.5]
    bg=[]
    for s in raw:
        if a.pose_convention.startswith('stored'):
            t=np.array(s['robot_pose']['base_T_gripper_m'])
        else:
            v=np.array(s['robot_pose']['raw'],dtype=float); t=np.eye(4); t[:3,3]=v[:3]
            t[:3,:3]=Rotation.from_quat([v[4],v[5],v[6],v[3]]).as_matrix()
        if a.pose_convention.endswith('inverse'): t=inv(t)
        bg.append(t)
    ct0=[np.array(s['camera_T_target_m']) for s in raw]
    board=manifest['board']; w=(board['inner_corners_cols']-1)*board['square_size_mm']/1000
    h=(board['inner_corners_rows']-1)*board['square_size_mm']/1000
    flip=np.eye(4); flip[:3,:3]=np.diag([-1.,-1.,1.]); flip[:3,3]=[w,h,0]
    ct1=[t@flip for t in ct0]
    rng=np.random.default_rng(42); best=None
    # Multi-start coordinate descent over the two valid checkerboard orientations.
    for start in range(80):
        bits=np.zeros(len(raw),dtype=int) if start==0 else rng.integers(0,2,len(raw))
        for _ in range(12):
            ct=[ct1[i] if bits[i] else ct0[i] for i in range(len(raw))]
            candidates=[]
            for name,flag in METHODS:
                try:
                    x=handeye(bg,ct,flag); candidates.append((score(bg,ct,x)[0],name,x))
                except Exception: pass
            if not candidates: break
            _,name,x=min(candidates,key=lambda z:z[0])
            boards=[bg[i]@x@(ct1[i] if bits[i] else ct0[i]) for i in range(len(raw))]
            m=mean_pose(boards); changed=False
            for i in range(len(raw)):
                e=[]
                for c in (ct0[i],ct1[i]):
                    te,re=errors([bg[i]@x@c],m); e.append(te[0]+2*re[0])
                nb=int(e[1]<e[0]); changed |= nb!=bits[i]; bits[i]=nb
            if not changed: break
        ct=[ct1[i] if bits[i] else ct0[i] for i in range(len(raw))]
        for name,flag in METHODS:
            try:
                x=handeye(bg,ct,flag); sc,te,re,m=score(bg,ct,x)
                item=(sc,name,x,bits.copy(),te,re,m)
                if best is None or sc<best[0]: best=item
            except Exception: pass
    if best is None: raise SystemExit('no solution')
    sc,name,x,bits,te,re,board_pose=best
    # Reject consistency outliers, then solve once more.
    keep=(te <= max(15.,np.median(te)+3*np.std(te))) & (re <= max(5.,np.median(re)+3*np.std(re)))
    bgk=[v for v,k in zip(bg,keep) if k]; ctk=[(ct1[i] if bits[i] else ct0[i]) for i,k in enumerate(keep) if k]
    final=[]
    for n,f in METHODS:
        try:
            xx=handeye(bgk,ctk,f); ss,tte,rre,bb=score(bgk,ctk,xx); final.append((ss,n,xx,tte,rre,bb))
        except Exception: pass
    sc,name,x,te,re,board_pose=min(final,key=lambda z:z[0])
    used=[raw[i]['id'] for i,k in enumerate(keep) if k]; rejected=[s['id'] for s in manifest['samples'] if s not in raw]+[raw[i]['id'] for i,k in enumerate(keep) if not k]
    q=Rotation.from_matrix(x[:3,:3]).as_quat()
    result={'schema_version':2,'calibration_mode':'eye_in_hand','camera_id':'wrist_d435i',
      'parent_frame':'tcp','child_frame':'d435i_color_optical_frame','method':name,'pose_convention':a.pose_convention,
      'gripper_T_camera_m':x.tolist(),'translation_m':x[:3,3].tolist(),'quaternion_xyzw':q.tolist(),
      'translation_rms_mm':float(np.sqrt(np.mean(te**2))),'translation_max_mm':float(np.max(te)),
      'rotation_rms_deg':float(np.sqrt(np.mean(re**2))),'rotation_max_deg':float(np.max(re)),
      'samples_used':used,'samples_rejected':sorted(set(rejected)),'checkerboard_flipped_ids':[raw[i]['id'] for i,b in enumerate(bits) if b],
      'quality_passed':bool(np.sqrt(np.mean(te**2))<10 and np.sqrt(np.mean(re**2))<2)}
    out=Path(a.out_dir); out.mkdir(parents=True,exist_ok=True)
    (out/'eye_in_hand_result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    np.savez(out/'eye_in_hand_matrix.npz',gripper_T_camera_m=x,quality_passed=result['quality_passed'])
    print(json.dumps(result,indent=2))

if __name__=='__main__': main()
