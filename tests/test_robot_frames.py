"""Independent FK oracle and frame invariance tests (truth used only as oracle)."""
import math
from pathlib import Path
import sys
import unittest

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from export_sensor_frames import DEFAULT_MODEL, export
from robot_frames import SensorKinematics, StateHistory, horizontal_to_root, rigid_inverse, transform


class SensorFramesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = mujoco.MjModel.from_xml_path(str(DEFAULT_MODEL))
        cls.fk = SensorKinematics()

    def test_generated_model_matches_source(self):
        self.assertEqual(self.fk.config, export(DEFAULT_MODEL))
        self.assertEqual([j['sdk_index'] for j in self.fk.waist], [12, 13, 14])

    def test_fk_matches_independent_mujoco_oracle(self):
        rng = np.random.default_rng(24)
        for _ in range(50):
            data = mujoco.MjData(self.model)
            joints = rng.uniform(-.4, .4, 29)
            root_q = rng.normal(size=4)
            root_q /= np.linalg.norm(root_q)
            data.qpos[:3] = rng.normal(size=3)
            data.qpos[3:7] = root_q
            for i, joint in enumerate(self.model.actuator_trnid[:, 0]):
                data.qpos[self.model.jnt_qposadr[joint]] = joints[i]
            mujoco.mj_forward(self.model, data)
            world_root = transform(data.xpos[self.model.body('pelvis').id], root_q)
            world_lidar = np.eye(4)
            site = self.model.site('livox_frame').id
            world_lidar[:3, :3] = data.site_xmat[site].reshape(3, 3)
            world_lidar[:3, 3] = data.site_xpos[site]
            np.testing.assert_allclose(
                self.fk.root_to_sensor(joints, 'lidar'),
                rigid_inverse(world_root) @ world_lidar, atol=1e-12)
            camera = self.model.camera('camera_color_optical_frame').id
            world_camera = np.eye(4)
            world_camera[:3, :3] = data.cam_xmat[camera].reshape(3, 3) @ np.diag([1, -1, -1])
            world_camera[:3, 3] = data.cam_xpos[camera]
            expected = horizontal_to_root(root_q) @ rigid_inverse(world_root) @ world_camera
            np.testing.assert_allclose(self.fk.horizontal_to_camera(joints, root_q), expected, atol=1e-12)

    def test_recognition_is_invariant_to_global_yaw(self):
        # Same body tilt and waist angles; changing global yaw cannot change recognition.
        joints = np.zeros(29)
        joints[12:15] = [.7, -.15, .2]
        roll_q = [math.cos(.15), math.sin(.15), 0, 0]
        expected = self.fk.horizontal_to_camera(joints, roll_q)
        for yaw in [0, math.pi/2, -math.pi/2, math.pi]:
            # q_z(yaw) * q_x(roll)
            c, s = math.cos(yaw/2), math.sin(yaw/2)
            a, b = roll_q[:2]
            actual = self.fk.horizontal_to_camera(joints, [c*a, c*b, s*b, s*a])
            np.testing.assert_allclose(actual, expected, atol=1e-12)

    def test_inverse_mount_and_waist_remove_lidar_motion(self):
        world_root = transform([3, -2, .8], [.8, .1, .2, .3])
        lidar_imu = transform([.006, -.012, .008], [.99, .01, -.02, .03])
        for yaw in [-1, 0, 1]:
            joints = np.zeros(29)
            joints[12:15] = [yaw, .2, -.1]
            world_imu = world_root @ self.fk.root_to_sensor(joints, 'lidar') @ lidar_imu
            recovered = self.fk.odometry_to_root(world_imu, joints, lidar_imu)
            np.testing.assert_allclose(recovered, world_root, atol=1e-12)
        np.testing.assert_allclose(self.fk.mounts['lidar'][:3, 2], [-.040131792565, 0, -.999194395113], atol=1e-10)

    def test_history_interpolates_measurements_and_rejects_stale_or_gap(self):
        h = StateHistory()
        h.add(1, np.zeros(29), [1, 0, 0, 0])
        h.add(1.02, np.ones(29), [-math.cos(.1), 0, 0, -math.sin(.1)])
        middle = h.at(1.01)
        np.testing.assert_allclose(middle.q, .5)
        np.testing.assert_allclose(middle.quaternion, [math.cos(.05), 0, 0, math.sin(.05)])
        with self.assertRaises(ValueError):
            h.at(2)
        h.add(2, np.ones(29), [1, 0, 0, 0])
        with self.assertRaises(ValueError):
            h.at(1.5)
        h.add(0, np.zeros(29), [1, 0, 0, 0])  # simulator reset, new epoch
        with self.assertRaises(ValueError):
            h.at(1)
        np.testing.assert_allclose(h.at(0).q, 0)


if __name__ == '__main__':
    unittest.main()
