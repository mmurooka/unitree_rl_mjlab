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
