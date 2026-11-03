#!/usr/bin/env python3
"""Replay fixed hand-eye calibration TCP poses.

Default is DRY-RUN only. It prints the stored TCP poses.
Use --execute only after confirming the workcell is safe and the robot/camera/board
layout matches the saved calibration session.

Pose format: x,y,z,qx,qy,qz,qw, translation in meters.
"""
import argparse
import json
import time
from pathlib import Path


def load_poses(path, only_used=True):
    data = json.load(open(path, encoding='utf-8'))
    poses = data.get('poses', [])
    if only_used:
        poses = [p for p in poses if p.get('used_in_best_calibration')]
    return data, poses


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--poses', default='/home/nvidia/calibration/fixed_calibration_poses/fixed_calibration_poses.json')
    ap.add_argument('--robot-ip', default='192.168.11.11')
    ap.add_argument('--all', action='store_true', help='include rejected samples too')
    ap.add_argument('--execute', action='store_true', help='actually move robot; currently requires project-specific pose move binding')
    ap.add_argument('--dwell', type=float, default=2.0, help='seconds to wait at each pose')
    args = ap.parse_args()

    data, poses = load_poses(Path(args.poses), only_used=not args.all)
    print('fixed calibration pose replay')
    print('robot_ip:', args.robot_ip)
    print('pose_order:', data.get('pose_order'))
    print('translation_unit:', data.get('translation_unit'))
    print('pose_count:', len(poses))
    print('execute:', args.execute)
    for i, p in enumerate(poses, 1):
        print('%02d %s sample=%s used=%s raw=%s' % (i, p.get('name'), p.get('sample_id'), p.get('used_in_best_calibration'), p.get('raw')))

    if not args.execute:
        print('\nDRY-RUN ONLY. No robot command was sent.')
        print('Next integration step: bind these robottarget TCP poses to the verified Zhenshi cartesian move command.')
        return

    raise SystemExit('EXECUTE is intentionally blocked until the project-specific cartesian pose move API is bound and verified.')


if __name__ == '__main__':
    main()
