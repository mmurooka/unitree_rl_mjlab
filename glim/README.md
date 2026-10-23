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
GLIM in a separate terminal. The supplied configuration values are included without
modification.

Run the following from the repository root. Replace `/path/to/unitree_rl_mjlab`
with the actual checkout location:

```bash
source /opt/ros/humble/setup.bash
cd /path/to/unitree_rl_mjlab
export G1_GLIM_CONFIG="$(realpath glim/config)"
ros2 run glim_ros glim_rosnode --ros-args -p config_path:="$G1_GLIM_CONFIG"
```

`realpath` produces an absolute path regardless of the checkout location. The
tutorial's template-copy step into `~/config` is unnecessary when using this
versioned configuration.

In the simulator and deploy terminals, run the same `export` from the repository
root and pass `--glim-config "$G1_GLIM_CONFIG"`. GLIM, simulator and deploy must
refer to the same directory.

`config.json` selects the sensor, ROS, preprocessing, estimation and mapping
settings. Deploy reads the sensor extrinsic and frame names to convert GLIM's
IMU pose into the pelvis pose. The simulator reads the same settings to match
simulated sensor placement and output. GLIM-free simulation needs no GLIM config.

See the [individual sim/real startup workflow](../doc/sim_real_workflow.md) for
component startup order and Xbox controls.
