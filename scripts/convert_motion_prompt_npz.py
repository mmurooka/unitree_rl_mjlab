import os
import time

import mjlab
import numpy as np
import torch
import tyro
from mjlab.scene import Scene
from mjlab.sim.sim import Simulation, SimulationCfg
from mjlab.utils.lab_api.math import (
    axis_angle_from_quat,
    quat_conjugate,
    quat_mul,
    quat_slerp,
)
from motion_prompt_npz import read_motion_prompt_fps

from src.tasks.tracking.config.g1.env_cfgs import unitree_g1_flat_tracking_env_cfg
from src.tasks.tracking.config.g1_23dof.env_cfgs import (
    unitree_g1_23dof_flat_tracking_env_cfg,
)


class MotionPromptLoader:
    def __init__(
        self,
        motion_file: str,
        output_fps: float,
        line_range: tuple[int, int] | None = None,
    ):
        self.motion_file = motion_file
        self.output_fps = output_fps
        self.output_dt = 1.0 / self.output_fps
        self.line_range = line_range
        self._load_motion()
        self._interpolate_motion()
        self._compute_velocities()

    def _load_motion(self):
        """Loads a raw MotionPrompt NPZ file."""
        with np.load(self.motion_file, allow_pickle=False) as data:
            self.input_fps = read_motion_prompt_fps(data)
            q_ref = np.asarray(data["q_ref"], dtype=np.float32)
            foot_contact = np.asarray(data["foot_contact"], dtype=np.float32)

        if self.line_range is not None:
            start = self.line_range[0] - 1
            end = self.line_range[1]
            q_ref = q_ref[start:end]
            foot_contact = foot_contact[start:end]

        self.motion_base_poss_input = torch.from_numpy(q_ref[:, :3])
        # Pinocchio free-flyer q uses [x, y, z, qx, qy, qz, qw].
        self.motion_base_rots_input = torch.from_numpy(q_ref[:, [6, 3, 4, 5]])
        self.motion_dof_poss_input = torch.from_numpy(q_ref[:, 7:])
        self.motion_contact_input = torch.from_numpy(foot_contact)

        self.input_frames = q_ref.shape[0]
        self.input_dt = 1.0 / self.input_fps
        self.duration = (self.input_frames - 1) * self.input_dt

    def _interpolate_motion(self):
        """Interpolates the motion to the output fps."""
        times = torch.arange(0, self.duration, self.output_dt, dtype=torch.float32)
        self.output_frames = times.shape[0]
        if self.output_frames < 3:
            raise ValueError(
                "Motion is too short to compute velocities at the output FPS (need at least 3 frames)"
            )
        index_0, index_1, blend = self._compute_frame_blend(times)
        self.motion_base_poss = self._lerp(
            self.motion_base_poss_input[index_0],
            self.motion_base_poss_input[index_1],
            blend.unsqueeze(1),
        )
        self.motion_base_rots = self._slerp(
            self.motion_base_rots_input[index_0],
            self.motion_base_rots_input[index_1],
            blend,
        )
        self.motion_dof_poss = self._lerp(
            self.motion_dof_poss_input[index_0],
            self.motion_dof_poss_input[index_1],
            blend.unsqueeze(1),
        )
        self.motion_contacts = self.motion_contact_input[index_0].clone()
        print(
            f"Motion interpolated, input frames: {self.input_frames}, "
            f"input fps: {self.input_fps}, "
            f"output frames: {self.output_frames}, "
            f"output fps: {self.output_fps}"
        )

    def _lerp(
        self, a: torch.Tensor, b: torch.Tensor, blend: torch.Tensor
    ) -> torch.Tensor:
        """Linear interpolation between two tensors."""
        return a * (1 - blend) + b * blend

    def _slerp(
        self, a: torch.Tensor, b: torch.Tensor, blend: torch.Tensor
    ) -> torch.Tensor:
        """Spherical linear interpolation between two quaternions."""
        slerped_quats = torch.zeros_like(a)
        for i in range(a.shape[0]):
            slerped_quats[i] = quat_slerp(a[i], b[i], float(blend[i]))
        return slerped_quats

    def _compute_frame_blend(
        self, times: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Computes the frame blend for the motion."""
        phase = times / self.duration
        index_0 = (phase * (self.input_frames - 1)).floor().long()
        index_1 = torch.minimum(index_0 + 1, torch.tensor(self.input_frames - 1))
        blend = phase * (self.input_frames - 1) - index_0
        return index_0, index_1, blend

    def _compute_velocities(self):
        """Computes the velocities of the motion."""
        self.motion_base_lin_vels = torch.gradient(
            self.motion_base_poss, spacing=self.output_dt, dim=0
        )[0]
        self.motion_dof_vels = torch.gradient(
            self.motion_dof_poss, spacing=self.output_dt, dim=0
        )[0]
        self.motion_base_ang_vels = self._so3_derivative(
            self.motion_base_rots, self.output_dt
        )

    def _so3_derivative(self, rotations: torch.Tensor, dt: float) -> torch.Tensor:
        """Computes the derivative of a sequence of SO3 rotations.

        Args:
          rotations: shape (B, 4).
          dt: time step.
        Returns:
          shape (B, 3).
        """
        q_prev, q_next = rotations[:-2], rotations[2:]
        q_rel = quat_mul(q_next, quat_conjugate(q_prev))  # shape (B−2, 4)

        omega = axis_angle_from_quat(q_rel) / (2.0 * dt)  # shape (B−2, 3)
        omega = torch.cat(
            [omega[:1], omega, omega[-1:]], dim=0
        )  # repeat first and last sample
        return omega


def create_conversion_scene(robot: str, output_fps: float):
    sim_cfg = SimulationCfg()
    sim_cfg.mujoco.timestep = 1.0 / output_fps
    if robot == "g1":  # 29 Dof
        scene = Scene(unitree_g1_flat_tracking_env_cfg().scene, device="cpu")
        joint_names = [
            "left_hip_pitch_joint",
            "left_hip_roll_joint",
            "left_hip_yaw_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
            "right_hip_pitch_joint",
            "right_hip_roll_joint",
            "right_hip_yaw_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",
            "waist_yaw_joint",
            "waist_roll_joint",
            "waist_pitch_joint",
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",
            "left_elbow_joint",
            "left_wrist_roll_joint",
            "left_wrist_pitch_joint",
            "left_wrist_yaw_joint",
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_wrist_roll_joint",
            "right_wrist_pitch_joint",
            "right_wrist_yaw_joint",
        ]
        output_dir = "./src/assets/motions/g1"
    elif robot == "g1_23dof":
        scene = Scene(unitree_g1_23dof_flat_tracking_env_cfg().scene, device="cpu")
        joint_names = [  # 23 Dof
            "left_hip_pitch_joint",
            "left_hip_roll_joint",
            "left_hip_yaw_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
            "right_hip_pitch_joint",
            "right_hip_roll_joint",
            "right_hip_yaw_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",
            "waist_yaw_joint",
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",
            "left_elbow_joint",
            "left_wrist_roll_joint",
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_wrist_roll_joint",
        ]
        output_dir = "./src/assets/motions/g1_23dof"
    else:
        raise ValueError(f"Unsupported robot: {robot}")

    model = scene.compile()

    sim = Simulation(num_envs=1, cfg=sim_cfg, model=model, device="cpu")

    scene.initialize(sim.mj_model, sim.model, sim.data)

    return sim, scene, joint_names, output_dir


class MotionPromptConverter:
    """Persistent CPU converter shared by the online worker and offline CLI."""

    def __init__(self, robot: str = "g1", output_fps: float = 50.0):
        from motion_prompt_fk import MotionPromptFK

        self.output_fps = output_fps
        self.sim, self.scene, self.joint_names, self.output_dir = (
            create_conversion_scene(robot, output_fps)
        )
        self.fk = MotionPromptFK(self.sim, self.scene, self.joint_names)

    def convert(
        self, input_file: str, output_file: str, *, line_range=None, render=False
    ):
        start = time.perf_counter()
        motion = MotionPromptLoader(input_file, self.output_fps, line_range=line_range)
        self.fk.convert(motion, output_file, render=render)
        print(
            f"MotionPrompt conversion finished in {time.perf_counter() - start:.3f} s: "
            f"{os.path.abspath(output_file)}",
            flush=True,
        )


def main(
    robot: str,
    input_file: str,
    output_name: str | None = None,
    output_fps: float = 50.0,
    output_path: str | None = None,
    render: bool = False,
    line_range: tuple[int, int] | None = None,
):
    """Replay raw MotionPrompt NPZ and output a train/deploy NPZ file.

    Args:
      input_file: Path to the input MotionPrompt NPZ file, including its fps field.
      output_name: Output filename under the robot's default motion directory.
      output_fps: Desired output frame rate.
      output_path: Exact output file path. Overrides output_name when specified.
      render: Render frames for inspection (no video file is saved).
      line_range: Range of input frames to process.
    """
    converter = MotionPromptConverter(robot, output_fps)
    if output_path is None:
        if output_name is None:
            raise ValueError("Specify either --output-name or --output-path.")
        if not output_name.endswith(".npz"):
            output_name += ".npz"
        output_path = os.path.join(converter.output_dir, output_name)
    elif not output_path.endswith(".npz"):
        output_path += ".npz"
    output_parent = os.path.dirname(os.path.abspath(output_path))
    os.makedirs(output_parent, exist_ok=True)

    converter.convert(input_file, output_path, line_range=line_range, render=render)


if __name__ == "__main__":
    tyro.cli(main, config=mjlab.TYRO_FLAGS)
