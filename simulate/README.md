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
Override these settings with `--disable-lidar` to disable MID-360 lidar and IMU,
or `--disable-camera` to disable D435 images and point clouds. Use both to
disable both sensor simulations:

```bash
./build/unitree_mujoco --disable-lidar --disable-camera
```

## Table and box scene

Select `src/assets/robots/unitree_g1/xmls/scene_g1_table_box.xml` in
`config.yaml` (`robot_scene`), or run:

```bash
./build/unitree_mujoco \
  --scene src/assets/robots/unitree_g1/xmls/scene_g1_table_box.xml
```

Set `robot_scene` back to `src/assets/robots/unitree_g1/xmls/scene_g1.xml`
for the flat environment. The table scene includes that same flat scene, so
robot and sensor definitions remain shared.

Each fixed table has a 0.40 × 0.55 m tabletop, 25 mm thick, supported by four
cylindrical legs of radius 20 mm. The top surface is 0.560 m above the floor.
Relative to the robot origin, the pickup table center is `(1.0, 0, 0)`
(1 m forward), and the destination table center is `(0.2, 1.0, 0)` (0.2 m forward, 1 m left).
The destination table is rotated 90 degrees counterclockwise about the vertical
axis. The robot must approach the pickup table before lifting the box.

The scene also includes ArUco ID 0 on the robot-facing side of the box and IDs
1 and 2 on top of the pickup and destination tables. `frame_pose_list.source`
in `config.yaml` selects either MuJoCo `ground_truth` or the `aruco` image
pipeline. See [`../aruco/README.md`](../aruco/README.md) for setup and real-camera
configuration.

The box is a free body with contact only (no attachment or scripted motion).
Its body dimensions are fixed at 0.26 × 0.35 × 0.19 m (x/y/z), matching the
physical box. The body mass is 0.1 kg. A rigid 0.26 × 0.42 × 0.015 m top plate
(mass 0.03 kg) has the same depth as the box and a 35 mm overhang on each
side to support the hands through contact.
Its initial center is `(0.87, 0, 0.657)`, with 2 mm clearance above the tabletop.
For reference generation, use `task_prompt.box.size=[0.26,0.35,0.19]`;
`task_prompt.box.grip_width_offset=-0.02` targets a 0.33 m hand spacing.
Positions and geometry dimensions can be edited directly in the scene XML;
MuJoCo box `size` values are half dimensions.

This defines the physical environment; successful lifting and transport with
the current policy have not been validated. Matching grip width alone does
not guarantee sufficient hand contact or grip force.

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
