"""Start external GLIM using this checkout's shared configuration."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(__file__).resolve().parents[1] / "config")
    return LaunchDescription([
        DeclareLaunchArgument("config_path", default_value=config,
                              description="Shared GLIM configuration directory"),
        Node(package="glim_ros", executable="glim_rosnode", output="screen",
             parameters=[{"config_path": LaunchConfiguration("config_path")}]),
    ])
