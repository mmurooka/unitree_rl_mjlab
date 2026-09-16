"""Run in the mjlab environment: metadata controls both resampling and velocity."""
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from convert_motion_prompt_npz import MotionPromptConverter, MotionPromptLoader


def write_prompt(path, fps, include_fps=True):
    time = np.arange(int(fps) + 1) / fps
    q = np.zeros((len(time), 36), dtype=np.float32)
    q[:, 0] = 0.4 * time
    q[:, 2] = 0.8
    q[:, 6] = 1
    q[:, 7] = 0.8 * time
    data = dict(format_version=1, q_ref=q, foot_contact=np.ones((len(time), 2)))
    if include_fps:
        data['fps'] = fps
    np.savez(path, **data)


class ConversionTests(unittest.TestCase):
    def test_loader_uses_stored_fps(self):
        with tempfile.TemporaryDirectory() as folder:
            path = str(Path(folder) / 'prompt.npz')
            for fps in (25.0, 50.0, 100.0, 200.0):
                with self.subTest(fps=fps):
                    write_prompt(path, fps)
                    motion = MotionPromptLoader(path, output_fps=50.0, device='cpu')
                    self.assertEqual(motion.input_fps, fps)
                    self.assertAlmostEqual(motion.duration, 1.0)
                    self.assertEqual(motion.output_frames, 50)
                    np.testing.assert_allclose(motion.motion_base_lin_vels[:, 0], 0.4, atol=1e-5)
                    np.testing.assert_allclose(motion.motion_dof_vels[:, 0], 0.8, atol=1e-5)
                    np.testing.assert_allclose(motion.motion_dof_poss[:, 0],
                                               np.arange(50) / 50 * 0.8, atol=1e-6)

    def test_missing_fps_does_not_fall_back_to_default(self):
        with tempfile.TemporaryDirectory() as folder:
            path = str(Path(folder) / 'prompt.npz')
            write_prompt(path, 200.0, include_fps=False)
            with self.assertRaisesRegex(ValueError, "missing 'fps'"):
                MotionPromptLoader(path, output_fps=50.0, device='cpu')

    def test_persistent_converter_uses_each_files_fps(self):
        converter = MotionPromptConverter('g1', 'cpu')
        with tempfile.TemporaryDirectory() as folder:
            path, output = str(Path(folder) / 'prompt.npz'), str(Path(folder) / 'policy.npz')
            for fps in (50.0, 200.0):
                with self.subTest(fps=fps):
                    write_prompt(path, fps)
                    converter.convert(path, output)
                    with np.load(output) as data:
                        self.assertEqual(data['fps'].item(), 50.0)
                        self.assertEqual(data['joint_pos'].shape, (50, 29))
                        np.testing.assert_allclose(data['joint_vel'][:, 0], 0.8, atol=1e-5)
                        np.testing.assert_allclose(data['body_lin_vel_w'][:, 0, 0], 0.4, atol=1e-5)


if __name__ == '__main__':
    unittest.main()
