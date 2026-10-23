#!/usr/bin/env python3
"""Offline, explicitly truth-assisted viewpoint calibration. Never sends commands.

Reconstruct a recorded carrying pose and render candidate fixed relative goals.
This is the one-time scene calibration allowed by the user, not a runtime goal
generator. Physical transport must subsequently be tested using frozen constants.
Run with the training Python (MuJoCo >= 3.5); EGL is suitable without a window.
"""
import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np
import yaml
from PIL import Image

from robot_frames import horizontal_to_root, rigid_inverse, transform

ROOT = Path(__file__).resolve().parents[1]


def yaw_pose(x, y, angle):
    return transform([x, y, 0], [math.cos(angle / 2), 0, 0, math.sin(angle / 2)])


def matrix_quaternion(rotation):
    result = np.zeros(4)
    mujoco.mju_mat2Quat(result, np.ascontiguousarray(rotation).reshape(9))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--entry-stamp', type=float, required=True)
    parser.add_argument('--posture-stamp', type=float, required=True)
    parser.add_argument('--back', type=float, nargs='+', default=[.15, .3, .45])
    parser.add_argument('--yaw', type=float, nargs='+', default=[1.4, 1.57, 1.75, 1.9, 2.094395])
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in [args.entry_stamp, args.posture_stamp, *args.back, *args.yaw]):
        parser.error('all candidate values and timestamps must be finite')
    args.output.mkdir(parents=True, exist_ok=False)
    observations = [json.loads(line) for line in (args.run / 'poses.jsonl').read_text().splitlines()]
    truth = [row['data'] for row in observations if row['topic'] == '/evaluation/frame_pose_list']
    joint_rows = (json.loads(line) for line in (args.run / 'joints.jsonl').read_text().splitlines())
    measured = [row for row in joint_rows if row['kind'] == 'measured']
    if not measured or any(row.get('tick_domain') != 'simulation' for row in measured):
        raise ValueError('requires simulated measured-state timestamps')
    if not truth or any(row.get('source') != 'evaluation_ground_truth' or
                        row.get('reference_frame') != 'root_yaw' for row in truth):
        raise ValueError('requires explicitly labeled root_yaw evaluation truth')
    model = mujoco.MjModel.from_xml_path(str(ROOT / 'src/assets/robots/unitree_g1/xmls/scene_g1_table_box_glim.xml'))
    # Match the recorded sensor aspect ratio; a 4:3 render changes visibility.
    config = yaml.safe_load((args.run / "sim/config.yaml").read_text())
    width, height = int(config["d435"]["width"]), int(config["d435"]["height"])
    if width <= 0 or height <= 0:
        raise ValueError("invalid recorded camera resolution")
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, height)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    table = model.body('pickup_table').id
    world_table = transform(data.xpos[table], data.xquat[table])

    def sample(stamp):
        row = min(truth, key=lambda r: abs(r['stamp'] - stamp))
        state = min(measured, key=lambda r: abs(r['tick_ms'] / 1000 - row['stamp']))
        if abs(row['stamp'] - stamp) > .06 or abs(state['tick_ms'] / 1000 - row['stamp']) > .03:
            raise ValueError('missing synchronized evaluation / joint samples')
        poses = {}
        for item in row['frame_pose_list']:
            p = item['rel_pose']
            poses[item['frame_name']] = transform(p[:3], [p[6], *p[3:6]])
        world_root = world_table @ rigid_inverse(poses['pickup_table'])
        return world_root, poses['carry_box'], state, row['stamp']

    entry, _, _, entry_stamp = sample(args.entry_stamp)
    actual, box, state, posture_stamp = sample(args.posture_stamp)
    q = np.asarray(state['q'])
    tilt = horizontal_to_root(state['quaternion_wxyz'])
    joint_addresses = model.jnt_qposadr[model.actuator_trnid[:, 0]]
    box_address = model.jnt_qposadr[model.joint('carry_box_free_joint').id]
    root_address = model.jnt_qposadr[model.joint('floating_base_joint').id]
    scene_option = mujoco.MjvOption()
    scene_option.geomgroup[:] = 1  # sensor rendering retains walls
    candidates = [('recorded', actual, None, None)]
    for back in args.back:
        for yaw in args.yaw:
            candidates.append((f'back_{back:.3f}_yaw_{yaw:.3f}', entry @ yaw_pose(-back, 0, yaw), back, yaw))
    results = []
    with mujoco.Renderer(model, height=height, width=width) as renderer:
        for name, pose, back, yaw in candidates:
            data.qpos[joint_addresses] = q
            robot = pose @ tilt
            carried_box = pose @ box
            for address, value in [(root_address, robot), (box_address, carried_box)]:
                data.qpos[address:address+3] = value[:3, 3]
                data.qpos[address+3:address+7] = matrix_quaternion(value[:3, :3])
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera='camera_color_optical_frame', scene_option=scene_option)
            Image.fromarray(renderer.render()).save(args.output / (name + '.png'))
            results.append(dict(image=name+'.png', back_m=back, yaw_rad=yaw,
                                world_root=pose.tolist()))
    (args.output / 'calibration.json').write_text(json.dumps(dict(
        source_run=str(args.run.resolve()), entry_stamp=entry_stamp, posture_stamp=posture_stamp,
        image_width=width, image_height=height,
        truth_use='one-time offline viewpoint calibration explicitly authorized by user',
        limitations='Frozen measured posture and box-to-root pose; no dynamics, slip, or collision validation. '
                     'Candidates are relative goal endpoints; verify actual sequential execution separately.',
        candidates=results), indent=2)+'\n')


if __name__ == '__main__':
    main()
