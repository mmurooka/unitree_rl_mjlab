"""Full-forward reference retained only for CPU converter regression tests."""

import numpy as np
import torch
from convert_motion_prompt_npz import MotionPromptLoader


def full_forward_reference(converter, source, output):
    motion = MotionPromptLoader(source, converter.output_fps)
    sim, scene = converter.sim, converter.scene
    robot = scene["robot"]
    indices = robot.find_joints(converter.joint_names, preserve_order=True)[0]
    keys = [
        "joint_pos",
        "joint_vel",
        "body_pos_w",
        "body_quat_w",
        "body_lin_vel_w",
        "body_ang_vel_w",
        "foot_contact",
    ]
    log = {k: [] for k in keys}
    scene.reset()
    for i in range(motion.output_frames):
        root = robot.data.default_root_state.clone()
        root[:, :3] = motion.motion_base_poss[i]
        root[:, :2] += scene.env_origins[:, :2]
        root[:, 3:7] = motion.motion_base_rots[i]
        root[:, 7:10] = motion.motion_base_lin_vels[i]
        root[:, 10:] = motion.motion_base_ang_vels[i]
        robot.write_root_state_to_sim(root)
        pos, vel = (
            robot.data.default_joint_pos.clone(),
            robot.data.default_joint_vel.clone(),
        )
        pos[:, indices] = motion.motion_dof_poss[i]
        vel[:, indices] = motion.motion_dof_vels[i]
        robot.write_joint_state_to_sim(pos, vel)
        sim.forward()
        scene.update(sim.mj_model.opt.timestep)
        values = [
            robot.data.joint_pos[0],
            robot.data.joint_vel[0],
            robot.data.body_link_pos_w[0],
            robot.data.body_link_quat_w[0],
            robot.data.body_link_lin_vel_w[0],
            robot.data.body_link_ang_vel_w[0],
            motion.motion_contacts[i],
        ]
        for k, value in zip(keys, values, strict=True):
            log[k].append(value.cpu().numpy().copy())
        torch.testing.assert_close(
            robot.data.body_link_lin_vel_w[0, 0], motion.motion_base_lin_vels[i]
        )
    np.savez(
        output, fps=[converter.output_fps], **{k: np.stack(v) for k, v in log.items()}
    )
