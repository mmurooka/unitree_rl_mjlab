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
            "--x",
            str(x),
            "--y",
            str(y),
            "--z",
            str(z),
            "--qx",
            str(qx),
            "--qy",
            str(qy),
            "--qz",
            str(qz),
            "--qw",
            str(qw),
            "--frame-id",
            parent,
            "--child-frame-id",
            child,
        ],
    )


def generate_launch_description():
    return LaunchDescription(
        [
            static_transform(
                "torso_to_camera",
                "torso_link",
                "camera_link",
                (0.0576235, 0.01753, 0.41987),
                (0.0, 0.4035452964, 0.0, 0.9149596678),
            ),
            static_transform(
                "torso_to_livox",
                "torso_link",
                "livox_frame",
                (0.0002835, 0.00003, 0.40618),
                (0.0, 0.9997985785, 0.0, -0.0200699388),
            ),
        ]
    )
