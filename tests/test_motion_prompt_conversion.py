"""Run in the mjlab environment: metadata controls both resampling and velocity."""

import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from convert_motion_prompt_npz import MotionPromptConverter, MotionPromptLoader, main
from motion_prompt_full_forward import full_forward_reference


def write_prompt(path, fps, include_fps=True):
    time = np.arange(int(fps) + 1) / fps
    q = np.zeros((len(time), 36), dtype=np.float32)
    q[:, 0] = 0.4 * time
    q[:, 2] = 0.8
    q[:, 6] = 1
    q[:, 7] = 0.8 * time
    data = dict(format_version=1, q_ref=q, foot_contact=np.ones((len(time), 2)))
    if include_fps:
        data["fps"] = fps
    np.savez(path, **data)


class ConversionTests(unittest.TestCase):
    def test_loader_uses_stored_fps(self):
        with tempfile.TemporaryDirectory() as folder:
            path = str(Path(folder) / "prompt.npz")
            for fps in (25.0, 50.0, 100.0, 200.0):
                with self.subTest(fps=fps):
                    write_prompt(path, fps)
                    motion = MotionPromptLoader(path, output_fps=50.0)
                    self.assertEqual(motion.input_fps, fps)
                    self.assertAlmostEqual(motion.duration, 1.0)
                    self.assertEqual(motion.output_frames, 50)
                    np.testing.assert_allclose(
                        motion.motion_base_lin_vels[:, 0], 0.4, atol=1e-5
                    )
                    np.testing.assert_allclose(
                        motion.motion_dof_vels[:, 0], 0.8, atol=1e-5
                    )
                    np.testing.assert_allclose(
                        motion.motion_dof_poss[:, 0],
                        np.arange(50) / 50 * 0.8,
                        atol=1e-6,
                    )

    def test_missing_fps_does_not_fall_back_to_default(self):
        with tempfile.TemporaryDirectory() as folder:
            path = str(Path(folder) / "prompt.npz")
            write_prompt(path, 200.0, include_fps=False)
            with self.assertRaisesRegex(ValueError, "missing 'fps'"):
                MotionPromptLoader(path, output_fps=50.0)

    def test_persistent_converter_uses_each_files_fps(self):
        converter = MotionPromptConverter("g1")
        with tempfile.TemporaryDirectory() as folder:
            path, output = (
                str(Path(folder) / "prompt.npz"),
                str(Path(folder) / "policy.npz"),
            )
            for fps in (50.0, 200.0):
                with self.subTest(fps=fps):
                    write_prompt(path, fps)
                    converter.convert(path, output)
                    with np.load(output) as data:
                        self.assertEqual(data["fps"].item(), 50.0)
                        self.assertEqual(data["joint_pos"].shape, (50, 29))
                        np.testing.assert_allclose(
                            data["joint_vel"][:, 0], 0.8, atol=1e-5
                        )
                        np.testing.assert_allclose(
                            data["body_lin_vel_w"][:, 0, 0], 0.4, atol=1e-5
                        )

    def test_offline_output_rate_and_frame_range(self):
        with tempfile.TemporaryDirectory() as folder:
            source = str(Path(folder) / "prompt.npz")
            output = str(Path(folder) / "nested" / "policy")
            write_prompt(source, 50.0)
            main("g1", source, output_fps=100.0, output_path=output, line_range=(6, 26))
            expected = MotionPromptLoader(source, 100.0, line_range=(6, 26))
            with np.load(output + ".npz") as actual:
                self.assertEqual(actual["fps"].item(), 100.0)
                np.testing.assert_array_equal(
                    actual["joint_pos"], expected.motion_dof_poss.numpy()
                )
                np.testing.assert_array_equal(
                    actual["joint_vel"], expected.motion_dof_vels.numpy()
                )
                np.testing.assert_allclose(
                    actual["body_pos_w"][:, 0],
                    expected.motion_base_poss.numpy(),
                    atol=1e-6,
                )

    def test_cpu_fk_matches_full_forward_for_rotating_motions(self):
        from scipy.spatial.transform import Rotation

        with tempfile.TemporaryDirectory() as folder:
            source = str(Path(folder) / "prompt.npz")
            fast = str(Path(folder) / "fast.npz")
            reference = str(Path(folder) / "reference.npz")
            for robot, dofs in (("g1", 29), ("g1_23dof", 23)):
                converter = MotionPromptConverter(robot)
                for fps in (50.0, 100.0):
                    with self.subTest(robot=robot, fps=fps):
                        t = np.arange(21) / fps
                        q = np.zeros((len(t), 7 + dofs), dtype=np.float32)
                        q[:, :3] = np.column_stack((0.2 * t, -0.1 * t, 0.8 + 0.03 * t))
                        q[:, 3:7] = Rotation.from_euler(
                            "xyz",
                            np.column_stack(
                                (0.2 + 0.1 * t, -0.15 + 0.2 * t, 0.8 + 0.7 * t)
                            ),
                        ).as_quat()
                        q[:, 7:] = 0.15 * np.sin(
                            t[:, None] * 2 + np.arange(dofs)[None, :]
                        )
                        contacts = np.column_stack((t < 0.1, t >= 0.1)).astype(float)
                        np.savez(
                            source,
                            format_version=1,
                            fps=fps,
                            q_ref=q,
                            foot_contact=contacts,
                        )
                        converter.convert(source, fast)
                        full_forward_reference(converter, source, reference)
                        with np.load(fast) as actual, np.load(reference) as expected:
                            self.assertEqual(set(actual.files), set(expected.files))
                            for key in expected.files:
                                self.assertEqual(actual[key].dtype, expected[key].dtype)
                                np.testing.assert_allclose(
                                    actual[key],
                                    expected[key],
                                    atol=1e-5,
                                    rtol=1e-5,
                                    err_msg=key,
                                )


if __name__ == "__main__":
    unittest.main()
