"""Evaluate MJCF sensor geometry independently of runtime localization."""
from pathlib import Path
import unittest

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


class Mid360ModelTest(unittest.TestCase):
    def test_fixed_extrinsic_and_acceleration_across_robot_poses(self):
        model = mujoco.MjModel.from_xml_path(str(
            ROOT / "src/assets/robots/unitree_g1/xmls/scene_g1.xml"))
        data = mujoco.MjData(model)
        lidar = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "livox_frame")
        imu = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "livox_imu")
        acc = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "livox_acc")
        # Freeze dynamics to evaluate static specific force, including inverted mount.
        for quaternion in ([1, 0, 0, 0], [.5, .5, .5, .5]):
            mujoco.mj_resetData(model, data)
            data.qpos[3:7] = quaternion
            mujoco.mj_kinematics(model, data)
            mujoco.mj_comPos(model, data)
            mujoco.mj_comVel(model, data)
            data.qacc[:] = 0
            mujoco.mj_rnePostConstraint(model, data)
            mujoco.mj_sensorAcc(model, data)
            rotation = data.site_xmat[lidar].reshape(3, 3)
            actual = rotation.T @ (data.site_xpos[imu] - data.site_xpos[lidar])
            np.testing.assert_allclose(actual, [.006, -.012, .008], atol=1e-12)
            np.testing.assert_allclose(data.site_xmat[imu], data.site_xmat[lidar], atol=1e-12)
            expected = data.site_xmat[imu].reshape(3, 3).T @ -model.opt.gravity
            offset = model.sensor_adr[acc]
            np.testing.assert_allclose(data.sensordata[offset:offset+3], expected, atol=1e-10)


if __name__ == "__main__":
    unittest.main()
