#!/usr/bin/env python3
"""Offline placement geometry from labeled evaluation truth; never used by control."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def evaluate(poses, events, window, table_top, table_half, box_half):
    completed = [json.loads(x) for x in events.read_text().splitlines()
                 if json.loads(x)['event'] == 'sequence_complete']
    if not completed:
        raise ValueError('Sequence did not complete; do not call placement success')
    finished_wall = completed[-1]['wall_time']
    samples = []
    for line in poses.read_text().splitlines():
        row = json.loads(line)
        if row['topic'] != '/evaluation/frame_pose_list':
            continue
        data = row['data']
        if data['source'] != 'evaluation_ground_truth' or data['reference_frame'] != 'root_yaw':
            raise ValueError('Expected explicitly labeled root_yaw evaluation truth')
        if row['received_wall'] < finished_wall:
            continue
        frames = {x['frame_name']: np.asarray(x['rel_pose']) for x in data['frame_pose_list']}
        table, box = frames['place_table'], frames['carry_box']
        if not np.isfinite(np.r_[table, box]).all():
            raise ValueError('Invalid evaluation pose')
        table_rot, box_rot = Rotation.from_quat(table[3:]), Rotation.from_quat(box[3:])
        position = table_rot.inv().apply(box[:3] - table[:3])
        orientation = (table_rot.inv() * box_rot).as_matrix()
        bottom_height = position[2] - np.abs(orientation[2]) @ box_half
        tilt = np.arccos(np.clip(orientation[2, 2], -1, 1))
        samples.append([data['stamp'], row['received_wall'], *position, bottom_height, tilt])
    if len(samples) < 2:
        raise ValueError('Not enough post-completion evaluation samples')
    data = np.array(samples)
    if np.any(np.diff(data[:, 0]) <= 0):
        raise ValueError('Split evaluation at clock resets')
    # Measure a full terminal window; all samples must be after sequence completion.
    if data[-1, 0] - data[0, 0] < window:
        raise ValueError('Not enough post-completion simulation time for requested window')
    data = data[data[:, 0] >= data[-1, 0] - window]
    if np.max(np.diff(data[:, 0])) > .2:
        raise ValueError('Evaluation gap exceeds 0.2 s')
    positions = data[:, 2:5]
    checks = dict(
        center_over_table=bool(np.all(np.abs(positions[:, :2]) <= table_half)),
        bottom_near_top=bool(np.max(np.abs(data[:, 5] - table_top)) <= .01),
        tilt_below_5_degrees=bool(np.max(data[:, 6]) <= np.deg2rad(5)),
        position_range_below_2mm=bool(np.max(np.ptp(positions, axis=0)) <= .002),
    )
    return dict(
        stable_placement_geometry=all(checks.values()), checks=checks,
        truth_use='offline evaluation only; no output is sent to control',
        limits='Geometric stability over this finite window, not contact-force measurement or repeated-run reliability',
        simulation_window_s=[float(data[0, 0]), float(data[-1, 0])], samples=len(data),
        final_box_in_table_m=positions[-1].tolist(), position_range_m=np.ptp(positions, axis=0).tolist(),
        maximum_bottom_error_m=float(np.max(np.abs(data[:, 5] - table_top))),
        maximum_tilt_deg=float(np.rad2deg(data[:, 6].max())),
        scene_dimensions=dict(table_top=table_top, table_half_xy=list(table_half), box_half_xyz=list(box_half)),
    )


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('poses', type=Path)
    p.add_argument('events', type=Path)
    p.add_argument('--window', type=float, default=5)
    p.add_argument('--table-top', type=float, required=True)
    p.add_argument('--table-half', type=float, nargs=2, required=True)
    p.add_argument('--box-half', type=float, nargs=3, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if not np.isfinite([a.window, a.table_top, *a.table_half, *a.box_half]).all() or min(a.window, *a.table_half, *a.box_half) <= 0:
        p.error('Expected finite dimensions and positive window/half sizes')
    result = evaluate(a.poses, a.events, a.window, a.table_top, np.array(a.table_half), np.array(a.box_half))
    a.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
