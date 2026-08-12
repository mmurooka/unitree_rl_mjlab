#!/usr/bin/env python3

from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    simulate_dir = Path(__file__).resolve().parents[1]

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(simulate_dir / "launch" / "g1_sensor_mounts.launch.py")
            )
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="unitree_mujoco_rviz",
            arguments=["-d", str(simulate_dir / "rviz" / "unitree_mujoco.rviz")],
            output="screen",
        ),
    ])
