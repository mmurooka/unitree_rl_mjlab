#!/usr/bin/env python3
"""Compare measured pelvis/torso/Livox gravity without any robot command publisher."""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from robot_frames import SensorKinematics, quaternion_rotation


def angle_deg(a, b):
    a, b = np.asarray(a), np.asarray(b)
    return float(np.degrees(np.arccos(np.clip(
        a @ b / np.linalg.norm(a) / np.linalg.norm(b), -1, 1))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--network', required=True)
    parser.add_argument('--domain-id', type=int, default=0)
    parser.add_argument('--duration', type=float, default=10)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 0 < args.duration <= 60:
        parser.error('duration must be in (0, 60] seconds')
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, IMUState_

    ChannelFactoryInitialize(args.domain_id, args.network)
    samples = dict(pelvis=[], torso=[], livox=[])

    def imu_values(msg):
        return dict(quaternion=list(msg.quaternion), acceleration=list(msg.accelerometer),
                    gyro=list(msg.gyroscope))

    def low(msg):
        now = time.monotonic()
        if samples['pelvis'] and now - samples['pelvis'][-1]['receipt'] < .01:
            return
        samples['pelvis'].append(dict(receipt=now, tick=msg.tick, mode_pr=msg.mode_pr,
                                      joints=[s.q for s in msg.motor_state[:29]], **imu_values(msg.imu_state)))

    def torso(msg):
        now = time.monotonic()
        if not samples['torso'] or now-samples['torso'][-1]['receipt'] >= .01:
            samples['torso'].append(dict(receipt=now, **imu_values(msg)))

    def lidar(msg):
        samples['livox'].append(dict(receipt=time.monotonic(),
            acceleration=[msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
            gyro=[msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]))

    rclpy.init()
    node = rclpy.create_node('g1_passive_mount_check')
    a = ChannelSubscriber('rt/lowstate', LowState_)
    b = ChannelSubscriber('rt/secondary_imu', IMUState_)
    try:
        a.Init(low, 10)
        b.Init(torso, 10)
        node.create_subscription(Imu, '/livox/imu', lidar, qos_profile_sensor_data)
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        a.Close()
        b.Close()
        node.destroy_node()
        rclpy.shutdown()
    if any(not values for values in samples.values()):
        raise RuntimeError('Need live pelvis, secondary torso and Livox IMU streams')
    # Use the common observation interval, avoiding startup-only samples.
    begin = max(values[0]['receipt'] for values in samples.values())
    end = min(values[-1]['receipt'] for values in samples.values())
    samples = {name: [v for v in values if begin <= v['receipt'] <= end]
               for name, values in samples.items()}
    if end-begin < 1 or any(not v for v in samples.values()):
        raise RuntimeError('Need at least one second of overlapping streams')
    kin = SensorKinematics()
    def gravity(q):
        return quaternion_rotation(q).T @ np.array([0., 0., 1.])
    predicted, torso_predicted = [], []
    for state in samples['pelvis']:
        root_lidar = kin.root_to_sensor(state['joints'], 'lidar')[:3, :3]
        root_torso = root_lidar @ kin.mounts['lidar'][:3, :3].T
        g = gravity(state['quaternion'])
        predicted.append(root_lidar.T @ g)
        torso_predicted.append(root_torso.T @ g)
    g_torso = np.mean([gravity(s['quaternion']) for s in samples['torso']], axis=0)
    g_livox = np.mean([s['acceleration'] for s in samples['livox']], axis=0)
    report = dict(
        common_duration_s=end-begin,
        counts={name: len(v) for name,v in samples.items()},
        mode_pr_values=sorted({s['mode_pr'] for s in samples['pelvis']}),
        torso_fk_vs_secondary_imu_deg=angle_deg(np.mean(torso_predicted,axis=0), g_torso),
        model_livox_gravity_error_deg=angle_deg(np.mean(predicted,axis=0), g_livox),
        torso_fk_gravity=np.mean(torso_predicted,axis=0).tolist(),
        torso_imu_gravity=g_torso.tolist(),
        predicted_livox_gravity=np.mean(predicted,axis=0).tolist(),
        measured_livox_gravity=(g_livox/np.linalg.norm(g_livox)).tolist(),
        max_gyro_rad_s={name: float(np.max(np.linalg.norm([s['gyro'] for s in v],axis=1)))
                        for name,v in samples.items()},
        interpretation='Static consistency check only. Gravity cannot identify mounting yaw or translation. No calibration is written.',
        samples=samples)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='samples'}, indent=2))


if __name__ == '__main__':
    main()
