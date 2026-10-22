#!/usr/bin/env python3
"""Offline root-odometry comparison against explicitly labeled evaluation truth.

Align map origins/yaw once at the first matched sample, then measure drift.
No evaluation data is published or passed back to goals or control.
"""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


def yaw(q):
    x, y, z, w = q
    return math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


def evaluate(poses, csv_paths, anchor_name, anchor_world):
    truth = {}
    for line in poses.read_text().splitlines():
        row = json.loads(line)
        if row['topic'] != '/evaluation/frame_pose_list':
            continue
        data = row['data']
        if data.get('source') != 'evaluation_ground_truth' or data.get('reference_frame') != 'root_yaw':
            raise ValueError('expected explicitly labeled root_yaw evaluation truth')
        p = next(item['rel_pose'] for item in data['frame_pose_list'] if item['frame_name'] == anchor_name)
        heading = anchor_world[2] - yaw(p[3:])
        c, s = math.cos(heading), math.sin(heading)
        truth[data['stamp']] = [anchor_world[0]-c*p[0]+s*p[1],
                                anchor_world[1]-s*p[0]-c*p[1], heading]
    if len(truth) < 2:
        raise ValueError('not enough evaluation samples')
    stamps = np.array(sorted(truth))
    values = np.array([truth[t] for t in stamps])
    values[:, 2] = np.unwrap(values[:, 2])
    estimates = {}
    sources = set()
    for path in csv_paths:
        with path.open() as stream:
            for row in csv.DictReader(stream):
                if row['localization_available'] != '1':
                    continue
                stamp = float(row['localization_stamp_s'])
                pose = [float(row[k]) for k in ('slam_pose_x_m', 'slam_pose_y_m', 'slam_pose_theta_rad')]
                if not np.isfinite([stamp, *pose]).all():
                    continue
                sources.add(row['source'])
                estimates[stamp] = pose
    if len(sources) != 1:
        raise ValueError('compare one explicit localization source per report')
    paired = []
    skipped = 0
    for t, estimate in sorted(estimates.items()):
        index = np.searchsorted(stamps, t)
        if index == 0 or index == len(stamps) or stamps[index]-stamps[index-1] > .15:
            skipped += 1
            continue
        actual = [np.interp(t, stamps, values[:, k]) for k in range(3)]
        paired.append((t, estimate, actual))
    if len(paired) < 2:
        raise ValueError('not enough time-matched odometry samples')
    est = np.array([x[1] for x in paired])
    gt = np.array([x[2] for x in paired])
    offset = gt[0, 2] - est[0, 2]
    c, s = math.cos(offset), math.sin(offset)
    rotation = np.array([[c, -s], [s, c]])
    aligned_xy = (est[:, :2]-est[0, :2]) @ rotation.T + gt[0, :2]
    position_error = np.linalg.norm(aligned_xy-gt[:, :2], axis=1)
    yaw_error = np.arctan2(np.sin(est[:, 2]+offset-gt[:, 2]), np.cos(est[:, 2]+offset-gt[:, 2]))
    return dict(source=next(iter(sources)), matched_samples=len(paired), skipped_samples=skipped,
                span_s=[paired[0][0], paired[-1][0]],
                alignment='single initial XY translation and yaw; no per-segment refitting',
                truth_use='offline evaluation only; fixed anchor world pose reconstructs root truth',
                anchor=dict(name=anchor_name, world_xy_yaw=list(anchor_world)),
                position_rmse_m=float(np.sqrt(np.mean(position_error**2))),
                position_max_m=float(position_error.max()),
                yaw_rmse_rad=float(np.sqrt(np.mean(yaw_error**2))),
                yaw_max_abs_rad=float(np.max(abs(yaw_error))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('poses', type=Path)
    parser.add_argument('csv', type=Path, nargs='+')
    parser.add_argument('--anchor-name', default='pickup_table')
    parser.add_argument('--anchor-world', type=float, nargs=3, required=True, metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = evaluate(args.poses, args.csv, args.anchor_name, args.anchor_world)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
