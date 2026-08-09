# Livox MID-360 scan pattern

`mid360.csv` is copied without modification from the official
[Livox laser simulation](https://github.com/Livox-SDK/livox_laser_simulation/blob/master/scan_mode/mid360.csv)
repository. The source revision used was `1cce1073633a062b92e30243a4c2920e45551bb5`.

The file is licensed under the MIT License in `LICENSE`.

## Simulation output

Build after sourcing ROS 2 so the optional publisher is enabled:

```bash
source /opt/ros/humble/setup.bash
cd simulate
mkdir -p build
cmake -S . -B build
cmake --build build -j
./build/unitree_mujoco
```

The simulator publishes the same ROS 2 topic and point layout as the default
MID-360 configuration of `livox_ros_driver2`:

- `/livox/lidar` (`sensor_msgs/msg/PointCloud2`, 10 Hz)
- `/livox/imu` (`sensor_msgs/msg/Imu`, 200 Hz)
- frame ID: `livox_frame`
- point fields: `x`, `y`, `z`, `intensity`, `tag`, `line`, `timestamp`

Start the configured RViz view in another terminal from the repository root:

```bash
rviz2 -d simulate/rviz/mid360.rviz
```

It uses `livox_frame` as the fixed frame and displays `/livox/lidar`, colored
by point height. No odometry, localization, or robot TF is published by this
component.

The current intensity is a fixed synthetic value and `tag` is zero. Rays are
cast incrementally at 200,000 points/s, so each cloud contains motion
distortion and each point has its simulated acquisition timestamp in ns.
