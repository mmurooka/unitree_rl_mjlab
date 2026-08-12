#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def static_transform(name, parent, child, translation, quaternion):
    x, y, z = translation
    qx, qy, qz, qw = quaternion
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=[
            "--x", str(x),
            "--y", str(y),
            "--z", str(z),
            "--qx", str(qx),
            "--qy", str(qy),
            "--qz", str(qz),
            "--qw", str(qw),
            "--frame-id", parent,
            "--child-frame-id", child,
        ],
    )


def generate_launch_description():
    return LaunchDescription([
        static_transform(
            "torso_to_camera",
            "torso_link",
            "camera_link",
            (0.0576235, 0.01753, 0.42987),
            (0.0, 0.4035452854, 0.0, 0.9149596727),
        ),
        static_transform(
            "torso_to_livox",
            "torso_link",
            "livox_frame",
            (0.0, 0.0, 0.405),
            (0.0, 1.0, 0.0, 0.0),
        ),
    ])
