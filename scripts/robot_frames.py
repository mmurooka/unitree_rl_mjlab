"""Model-based G1 transforms; no simulator state, ROS TF, or global position.

T_A_B maps B coordinates into A coordinates. Quaternion inputs use SDK wxyz.
The root_yaw origin is pelvis, its +X is the horizontal projection of pelvis +X,
+Y points left and +Z up. Its yaw is not the torso/LiDAR heading.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from pathlib import Path
import math
import threading

import numpy as np
import yaml

DEFAULT_MODEL = Path(__file__).resolve().parents[1] / 'config/g1_sensor_frames.yaml'


def quaternion_rotation(q) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-9:
        raise ValueError('expected a finite nonzero wxyz quaternion')
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def transform(translation, quaternion_wxyz) -> np.ndarray:
    out = np.eye(4)
    translation = np.asarray(translation, dtype=float)
    if translation.shape != (3,) or not np.isfinite(translation).all():
        raise ValueError('expected finite translation xyz')
    out[:3, :3] = quaternion_rotation(quaternion_wxyz)
    out[:3, 3] = translation
    return out


def rigid_inverse(pose: np.ndarray) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = pose[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ pose[:3, 3]
    return result


def horizontal_to_root(root_quaternion_wxyz) -> np.ndarray:
    rotation = quaternion_rotation(root_quaternion_wxyz)
    if np.linalg.norm(rotation[:2, 0]) < 1e-6:
        raise ValueError('root forward axis has no horizontal projection')
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    c, s = math.cos(yaw), math.sin(yaw)
    out = np.eye(4)
    out[:3, :3] = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]]) @ rotation
    return out


class SensorKinematics:
    def __init__(self, path: Path = DEFAULT_MODEL):
        self.config = yaml.safe_load(path.read_text())
        if self.config['schema_version'] != 1 or self.config['root_frame'] != 'pelvis':
            raise ValueError('unsupported sensor kinematics schema/root')
        self.waist = self.config['waist']
        self.mounts = {}
        for name in ('lidar', 'color_optical'):
            pose = np.asarray(self.config['torso_to_' + name], dtype=float)
            if (pose.shape != (4, 4) or not np.isfinite(pose).all()
                    or not np.allclose(pose[3], [0, 0, 0, 1])
                    or not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3))
                    or not np.isclose(np.linalg.det(pose[:3, :3]), 1)):
                raise ValueError('invalid fixed sensor transform')
            self.mounts[name] = pose

    def root_to_sensor(self, measured_sdk_q, sensor: str) -> np.ndarray:
        q = np.asarray(measured_sdk_q, dtype=float)
        if q.shape != (29,) or not np.isfinite(q).all():
            raise ValueError('expected 29 finite measured SDK joint angles')
        pose = np.eye(4)
        for joint in self.waist:
            pose = pose @ transform(joint['translation'], joint['quaternion_wxyz'])
            axis = np.asarray(joint['axis'], dtype=float)
            axis /= np.linalg.norm(axis)
            angle = q[joint['sdk_index']]
            rotation = quaternion_rotation(np.r_[math.cos(angle/2), axis*math.sin(angle/2)])
            pivot = np.asarray(joint['pivot'])
            hinge = np.eye(4)
            hinge[:3, :3] = rotation
            hinge[:3, 3] = pivot - rotation @ pivot
            pose = pose @ hinge
        return pose @ self.mounts[sensor]

    def horizontal_to_camera(self, measured_sdk_q, root_quaternion_wxyz):
        return horizontal_to_root(root_quaternion_wxyz) @ self.root_to_sensor(
            measured_sdk_q, 'color_optical')

    def odometry_to_root(self, odometry_to_imu, measured_sdk_q, lidar_to_imu):
        root_to_imu = self.root_to_sensor(measured_sdk_q, 'lidar') @ lidar_to_imu
        return odometry_to_imu @ rigid_inverse(root_to_imu)


@dataclass(frozen=True)
class MeasuredState:
    stamp: float
    q: np.ndarray
    quaternion: np.ndarray


class StateHistory:
    """Bounded, thread-safe measured states interpolated at sensor timestamps.

    No extrapolation beyond tolerance; clock rewinds discard the old epoch.
    Caller must supply stamps in the same clock domain as camera/odometry.
    """
    def __init__(self, duration=5.0, tolerance=0.03):
        if not math.isfinite(duration) or not math.isfinite(tolerance) or duration <= 0 or tolerance < 0:
            raise ValueError('invalid history limits')
        self.duration, self.tolerance = duration, tolerance
        self.samples = deque()
        self.lock = threading.Lock()

    def add(self, stamp, q, quaternion):
        q = np.asarray(q, dtype=float).copy()
        quaternion = np.asarray(quaternion, dtype=float).copy()
        quaternion_rotation(quaternion)  # validate
        if not math.isfinite(stamp) or q.shape != (29,) or not np.isfinite(q).all():
            raise ValueError('invalid measured state')
        quaternion /= np.linalg.norm(quaternion)
        sample = MeasuredState(stamp, q, quaternion)
        with self.lock:
            if self.samples and stamp < self.samples[-1].stamp:
                self.samples.clear()
            if self.samples and stamp == self.samples[-1].stamp:
                self.samples[-1] = sample
            else:
                self.samples.append(sample)
            while len(self.samples) > 1 and self.samples[0].stamp < stamp - self.duration:
                self.samples.popleft()

    def at(self, stamp) -> MeasuredState:
        if not math.isfinite(stamp):
            raise ValueError('invalid measurement timestamp')
        with self.lock:
            samples = list(self.samples)
        if not samples:
            raise ValueError('waiting for measured robot state')
        if stamp < samples[0].stamp - self.tolerance or stamp > samples[-1].stamp + self.tolerance:
            raise ValueError('sensor timestamp outside measured-state history')
        if stamp <= samples[0].stamp:
            return samples[0]
        for left, right in zip(samples, samples[1:]):
            if stamp == right.stamp:
                return right
            if stamp < right.stamp:
                if right.stamp - left.stamp > 2 * self.tolerance:
                    raise ValueError('gap in measured-state history')
                t = (stamp-left.stamp)/(right.stamp-left.stamp)
                a, b = left.quaternion, right.quaternion
                dot = float(a @ b)
                if dot < 0:
                    b, dot = -b, -dot
                if dot > .9995:
                    q = (1-t)*a + t*b
                else:
                    angle = math.acos(np.clip(dot, -1, 1))
                    q = (math.sin((1-t)*angle)*a + math.sin(t*angle)*b)/math.sin(angle)
                return MeasuredState(stamp, (1-t)*left.q+t*right.q, q/np.linalg.norm(q))
        return samples[-1]
