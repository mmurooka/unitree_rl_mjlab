#!/usr/bin/env python3
"""Read-only sensor audit. Never launches drivers, controllers or command publishers."""
import argparse
import json
from pathlib import Path
import struct
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=10.)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--environment', choices=['sim', 'real'], required=True)
    parser.add_argument('--network', help='optionally subscribe to measured Unitree LowState')
    parser.add_argument('--domain-id', type=int, default=0, help='Unitree DDS domain (independent of ROS)')
    args = parser.parse_args()
    if not 0 < args.duration <= 300:
        parser.error('duration must be between 0 and 300 seconds')
    rclpy.init()
    node = rclpy.create_node('g1_read_only_sensor_audit')
    state = None
    subscriptions = []
    samples, details, accelerations = {}, {}, []
    contracts = {
        '/camera/camera/color/image_raw': Image,
        '/camera/camera/color/camera_info': CameraInfo,
        '/livox/imu': Imu, '/livox/lidar': PointCloud2,
        '/glim_ros/odom': Odometry, '/frame_pose_list': String,
    }

    def receive(topic, message):
        now = time.monotonic()
        if isinstance(message, String):
            value = json.loads(message.data)
            stamp = value.get('stamp', 0.)
            details[topic] = value
        else:
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            item = {'frame': message.header.frame_id}
            if isinstance(message, (Image, CameraInfo)):
                item.update(width=message.width, height=message.height)
                if isinstance(message, CameraInfo):
                    item.update(k=list(message.k), d=list(message.d))
            elif isinstance(message, Imu):
                accelerations.append([message.linear_acceleration.x,
                                      message.linear_acceleration.y,
                                      message.linear_acceleration.z])
            elif isinstance(message, PointCloud2):
                item.update(points=message.width * message.height, point_step=message.point_step,
                            fields=[(f.name, f.offset, f.datatype) for f in message.fields])
                for field in message.fields:
                    if field.name == 'timestamp' and field.datatype == 8 and message.width:
                        endian = '>' if message.is_bigendian else '<'
                        first = struct.unpack_from(endian+'d', message.data, field.offset)[0]
                        last = struct.unpack_from(endian+'d', message.data,
                                                  (message.width-1)*message.point_step+field.offset)[0]
                        item.update(first_point_timestamp=first, last_point_timestamp=last,
                                    first_point_minus_header_s=first*1e-9-stamp,
                                    point_span_s=(last-first)*1e-9)
            elif isinstance(message, Odometry):
                item.update(child_frame=message.child_frame_id,
                            position=[message.pose.pose.position.x, message.pose.pose.position.y,
                                      message.pose.pose.position.z])
            details[topic] = item
        samples.setdefault(topic, []).append((now, stamp, time.time()))

    report = {'environment': args.environment, 'duration_s': args.duration, 'topics': {}}
    try:
        if args.network:
            from measured_robot_state import MeasuredRobotState
            state = MeasuredRobotState(args.environment, time.time, args.network, args.domain_id)
        for topic, msg_type in contracts.items():
            subscriptions.append(node.create_subscription(
                msg_type, topic, lambda msg, topic=topic: receive(topic, msg), qos_profile_sensor_data))
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        for topic in contracts:
            values = np.asarray(samples.get(topic, []))
            item = dict(count=len(values), publishers=node.count_publishers(topic),
                        latest=details.get(topic))
            if len(values) > 1:
                item.update(received_hz=(len(values)-1)/(values[-1, 0]-values[0, 0]),
                            rewinds=int(np.sum(np.diff(values[:, 1]) < 0)))
                if args.environment == 'real':
                    item['mean_receipt_minus_stamp_s'] = float(np.mean(values[:, 2]-values[:, 1]))
            report['topics'][topic] = item
        if accelerations:
            report['livox_acceleration_mean_g'] = np.mean(accelerations, axis=0).tolist()
            report['livox_acceleration_norm_g'] = float(np.linalg.norm(accelerations, axis=1).mean())
        if state:
            with state.history.lock:
                latest = state.history.samples[-1] if state.history.samples else None
            if latest and time.monotonic() - state.last_received < .5:
                from robot_frames import SensorKinematics, quaternion_rotation
                rotation = quaternion_rotation(latest.quaternion) @ SensorKinematics().root_to_sensor(
                    latest.q, 'lidar')[:3, :3]
                report['measured_state'] = dict(
                    quaternion_wxyz=latest.quaternion.tolist(), waist_q=latest.q[12:15].tolist(),
                    predicted_static_force_in_lidar_g=(rotation.T @ np.array([0., 0., 1.])).tolist())
                report['mount_check_limit'] = (
                    'Static-force prediction in lidar axes, not a full extrinsic calibration. '
                    'Compare to Livox IMU only when IMU/lidar rotation is identity. '
                    'Gravity alone cannot determine yaw or translation.')
            else:
                report['measured_state_error'] = state.last_error or 'no fresh LowState'
    finally:
        if state:
            state.close()
        node.destroy_node()
        rclpy.shutdown()
    encoded = json.dumps(report, indent=2)
    if args.output:
        args.output.write_text(encoded+'\n')
    print(encoded)


if __name__ == '__main__':
    main()
