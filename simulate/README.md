# Unitree MuJoCo simulator

This directory contains the MuJoCo-based sim2sim environment. It emulates the
Unitree low-level interface and can publish simulated Livox MID-360 and Intel
RealSense D435 data over ROS 2.

## Build and run

Source ROS 2 before configuring CMake so that the sensor publishers are built:

```bash
source /opt/ros/humble/setup.bash
cd simulate
cmake -S . -B build
cmake --build build -j
./build/unitree_mujoco
```

Runtime settings, including sensor rates and topic names, are in `config.yaml`.

## ROS 2 interfaces

The MID-360 simulation publishes:

- `/livox/lidar` (`sensor_msgs/msg/PointCloud2`, 10 Hz)
- `/livox/imu` (`sensor_msgs/msg/Imu`, 200 Hz)
- frame: `livox_frame`

Rays are acquired incrementally, so a cloud contains motion distortion. The
per-point `timestamp` is in seconds relative to the cloud header timestamp.
Intensity is synthetic and `tag` is zero.

The D435 simulation publishes RealSense-compatible interfaces under
`/camera/camera`:

- `color/image_raw` and `color/camera_info`
- `depth/image_rect_raw` and `depth/camera_info`
- `aligned_depth_to_color/image_raw` and its `camera_info`
- `depth/color/points`
- RealSense-internal transforms below `camera_link` on `/tf_static`

Color is `rgb8`. Depth is idealized `16UC1` in millimeters, with zero for an
invalid measurement. The simulated camera has no lens distortion or noise.
`pointcloud_downsample` applies a pixel stride to point-cloud generation while
leaving the RGB and depth images at their configured resolution.

Sensor mounting transforms are intentionally separate from the sensor drivers.
To publish them without RViz, run:

```bash
ros2 launch $(pwd)/launch/g1_sensor_mounts.launch.py
```

This publishes `torso_link -> livox_frame` and
`torso_link -> camera_link`. A complete robot application should eventually
provide these transforms from its URDF and `robot_state_publisher`.

Use Best Effort reliability for the image and point-cloud displays in RViz.
Start the sensor mounting transforms and the shared MID-360/D435 RViz
configuration together with:

```bash
ros2 launch $(pwd)/launch/unitree_mujoco_rviz.launch.py
```

## Third-party assets

`assets/livox/mid360.csv` is copied without modification from the official
[Livox laser simulation](https://github.com/Livox-SDK/livox_laser_simulation/blob/master/scan_mode/mid360.csv)
repository at revision `1cce1073633a062b92e30243a4c2920e45551bb5`.
It is distributed under the MIT license in `assets/livox/LICENSE`.

See `assets/holodeck_objaverse_val_1/README.md` and `ATTRIBUTIONS.json` for the
office environment provenance and licenses.
