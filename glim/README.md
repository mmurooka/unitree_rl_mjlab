# GLIM and Livox MID-360 setup

Follow the [GLIM + Livox MID360 tutorial](https://zenn.dev/smrg_aist/articles/144e97133234e2)
for installation, network configuration and connection checks. This repository
uses the driver build command and shared GLIM configuration described below.

## Build the Livox ROS driver

After preparing Livox SDK2 and `livox_ros_driver2` as described in the tutorial,
run this command from the ROS workspace root (the parent of `src/`):

```bash
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
  --symlink-install
```

Apply `PYTHONNOUSERSITE=1` to this build invocation. The CMake option name is
`DISTRO_ROS`. After building, source that workspace's `install/setup.bash` before
launching the driver. Configure `MID360_config.json` for the actual LiDAR and
host IP addresses as described in the tutorial.

## Shared configuration and GLIM startup

The versioned [`config/`](config/) directory contains the shared configuration for
sim and real. GLIM itself and the Livox driver are external installations; launch
GLIM in a separate terminal.

Both sim and real explicitly use `livox_imu` for GLIM's IMU/base output frame
and `livox_frame` for its LiDAR output frame. Thus `/glim_ros/odom` describes
the IMU pose with child frame `livox_imu`; Navigation reads this name from the
same configuration and converts that pose to the pelvis pose.
Raw Livox messages can still label both streams `livox_frame`; the GLIM output
names do not rewrite those input headers or change the sensor geometry.
IMU-to-LiDAR TF publication remains disabled because recognition and Navigation
compute sensor transforms without this TF. GLIM still uses `T_lidar_imu`
internally. Restart GLIM and deploy together after changing frame settings.

Run the following from the repository root. Replace `/path/to/unitree_rl_mjlab`
with the actual checkout location:

```bash
source /opt/ros/humble/setup.bash
cd /path/to/unitree_rl_mjlab
ros2 launch glim/launch/glim.launch.py
```

The launch file resolves `glim/config` relative to its own location.
`scripts/run_online_deploy.py` uses the same directory by default. No environment
variable or configuration-path argument is required. Copying templates into
`~/config` is unnecessary.

For a custom configuration, pass `config_path:=/absolute/path` to the launch
command and `--glim-config /absolute/path` to deploy. GLIM itself remains an
external installation, started separately.

`config.json` selects the sensor, ROS, preprocessing, estimation and mapping
settings. Deploy reads the sensor extrinsic and frame names to convert GLIM's
IMU pose into the pelvis pose.

The simulator never reads GLIM settings. MJCF defines the inverted LiDAR mount
and separate IMU site, with a fixed lidar-to-IMU translation of (6, -12, 8) mm
and identity relative rotation. Simulation transport settings define the Livox
topics and frame labels; acceleration is published in g.
GLIM calibration must describe this geometry, just as it must describe real
hardware. Changing GLIM calibration does not change simulated hardware.
This separation makes calibration mismatches observable in simulation.
The retained offset matches the supplied calibration; actual hardware accuracy
still requires hardware validation.

See the [individual sim/real startup workflow](../doc/sim_real_workflow.md) for
component startup order and Xbox controls.

## Livox output contract

Use the tutorial's external Livox driver and its `rviz_MID360_launch.py`
(`xfer_format=0`, PointCloud2). Source your own Livox workspace first:

```bash
source /opt/ros/humble/setup.bash
source /path/to/ws_livox/install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

Set the LiDAR and host addresses in that driver's `MID360_config.json` for
your network. Keep `extrinsic_parameter` zero: raw sensor coordinates are
converted through the shared robot model, not pre-rotated in the driver.

Both sim and real publish `/livox/lidar` as PointCloud2 with x/y/z/intensity,
tag, line and float64 `timestamp`. Each point timestamp is **absolute nanoseconds**;
ROS header stamps remain sec/nanosec. In sim the absolute time is the simulation
clock, not wall time. The shared GLIM configuration explicitly selects
`perpoint_relative_time=false` and `perpoint_time_scale=1e-9`, with auto-detection
disabled. IMU acceleration is in g; shared `acc_scale=9.80665` converts to m/s².
Rebuild the simulator and restart GLIM together after updating this contract.

The driver can select CustomMsg with `xfer_format=1`, but this changes the whole
message type and is not this workflow. The inspected driver has no separate
PointCloud2 setting for seconds vs nanoseconds or absolute vs relative time.

For repeatable sensor-only checks and the remaining mount discrepancy, see
[sensor validation](../doc/real_sensor_validation.md).
