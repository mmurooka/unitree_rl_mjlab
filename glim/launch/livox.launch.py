"""Start the external MID360 driver as PointCloud2, without RViz."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory("livox_ros_driver2")) /
                 "config/MID360_config.json")
    return LaunchDescription([
        DeclareLaunchArgument("user_config_path", default_value=config,
                              description="External driver config with sensor/host IPs"),
        Node(package="livox_ros_driver2", executable="livox_ros_driver2_node",
             name="livox_lidar_publisher", output="screen", parameters=[{
                 "xfer_format": 0, "multi_topic": 0, "data_src": 0,
                 "publish_freq": 10.0, "output_data_type": 0,
                 "frame_id": "livox_frame",
                 "user_config_path": LaunchConfiguration("user_config_path"),
             }]),
    ])
