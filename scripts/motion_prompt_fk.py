"""CPU reference kinematics without collision detection or dynamics solving."""

import mujoco
import numpy as np
from mjlab.utils.lab_api.math import quat_apply_inverse


class MotionPromptFK:
    def __init__(self, sim, scene, joint_names):
        self.model = sim.mj_model
        self.data = mujoco.MjData(self.model)
        robot = scene["robot"]
        indexing = robot.data.indexing
        self.body_ids = indexing.body_ids.cpu().numpy()
        self.root_body_id = indexing.root_body_id
        self.joint_q = indexing.joint_q_adr.cpu().numpy()
        self.joint_v = indexing.joint_v_adr.cpu().numpy()
        self.root_q = indexing.free_joint_q_adr.cpu().numpy()
        self.root_v = indexing.free_joint_v_adr.cpu().numpy()
        self.joint_order = robot.find_joints(joint_names, preserve_order=True)[0]
        self.default_pos = robot.data.default_joint_pos[0].cpu().numpy().copy()
        self.default_vel = robot.data.default_joint_vel[0].cpu().numpy().copy()
        self.origin = scene.env_origins[0].cpu().numpy().copy()

    def convert(self, motion, output_file, *, render=False):
        count = motion.output_frames
        positions = motion.motion_base_poss.cpu().numpy().copy()
        positions[:, :2] += self.origin[:2]
        rotations = motion.motion_base_rots.cpu().numpy()
        linear = motion.motion_base_lin_vels.cpu().numpy()
        angular = (
            quat_apply_inverse(motion.motion_base_rots, motion.motion_base_ang_vels)
            .cpu()
            .numpy()
        )
        joint_pos = np.tile(self.default_pos, (count, 1))
        joint_vel = np.tile(self.default_vel, (count, 1))
        joint_pos[:, self.joint_order] = motion.motion_dof_poss.cpu().numpy()
        joint_vel[:, self.joint_order] = motion.motion_dof_vels.cpu().numpy()
        shape = (count, len(self.body_ids))
        log = {
            "fps": [motion.output_fps],
            "joint_pos": joint_pos,
            "joint_vel": joint_vel,
            "body_pos_w": np.empty((*shape, 3), dtype=np.float32),
            "body_quat_w": np.empty((*shape, 4), dtype=np.float32),
            "body_lin_vel_w": np.empty((*shape, 3), dtype=np.float32),
            "body_ang_vel_w": np.empty((*shape, 3), dtype=np.float32),
            "foot_contact": motion.motion_contacts.cpu().numpy(),
        }
        mujoco.mj_resetData(self.model, self.data)
        renderer = (
            mujoco.Renderer(self.model, height=480, width=640) if render else None
        )
        try:
            for frame in range(count):
                self.data.qpos[self.root_q[:3]] = positions[frame]
                self.data.qpos[self.root_q[3:]] = rotations[frame]
                self.data.qpos[self.joint_q] = joint_pos[frame]
                self.data.qvel[self.root_v[:3]] = linear[frame]
                self.data.qvel[self.root_v[3:]] = angular[frame]
                self.data.qvel[self.joint_v] = joint_vel[frame]
                # The velocity recursion requires the COM quantities from comPos.
                mujoco.mj_kinematics(self.model, self.data)
                mujoco.mj_comPos(self.model, self.data)
                mujoco.mj_comVel(self.model, self.data)
                pos = self.data.xpos[self.body_ids]
                cvel = self.data.cvel[self.body_ids]
                offset = pos - self.data.subtree_com[self.root_body_id]
                log["body_pos_w"][frame] = pos
                log["body_quat_w"][frame] = self.data.xquat[self.body_ids]
                log["body_ang_vel_w"][frame] = cvel[:, :3]
                log["body_lin_vel_w"][frame] = cvel[:, 3:] + np.cross(
                    cvel[:, :3], offset
                )
                if renderer is not None:
                    renderer.update_scene(self.data)
                    renderer.render()
        finally:
            if renderer is not None:
                renderer.close()
        np.savez(output_file, **log)
