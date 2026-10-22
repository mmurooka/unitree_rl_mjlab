# Sim/real sensor and navigation workflow

GLIM is an external program. Use the same installed GLIM version and the same
complete configuration directory in sim and real. The controller and simulator
read that directory to identify the IMU frame and its LiDAR extrinsic; neither
starts GLIM, copies its settings, nor writes its files. `SLAM/GLIM_config` is only
a ROS-settings reference, not a complete installation or the active config path.

The commands below run from this repository, except the explicitly indicated
TaskPromptRL command. Source the installed ROS environment in every ROS terminal
(e.g. `/opt/ros/humble/setup.bash`). Use a ROS-compatible Python environment with
OpenCV ArUco, NumPy, PyYAML and the official Unitree SDK2 Python for recognition.
Use the existing motion-conversion Python environment for `run_online_deploy.py`.

## GLIM sim2sim

Set the external path consistently in each terminal:

```bash
export G1_GLIM_CONFIG=/absolute/path/to/your/external/glim/config
```

1. Start the simulator (its ROS sensor outputs begin, but no GLIM process starts):

```bash
simulate/build/unitree_mujoco --network lo --glim-config "$G1_GLIM_CONFIG" \
  --scene src/assets/robots/unitree_g1/xmls/scene_g1_table_box_glim.xml
```

2. Start deploy in another terminal:

```bash
python scripts/run_online_deploy.py --network lo --environment sim \
  --navigation-localization-source glim --glim-config "$G1_GLIM_CONFIG"
```

3. Enter FixStand, then **Velocity, confirm it is active, then release the hang**.
   Wait for a stable stance. Do not release the hang before Velocity is active.
4. Start external GLIM using its normal command in another terminal:

```bash
ros2 run glim_ros glim_rosnode --ros-args -p config_path:="$G1_GLIM_CONFIG"
```

5. Start recognition independently:

```bash
python aruco/frame_pose_publisher.py --environment sim --network lo
```

6. Once GLIM is initialized and the marker is visible, send a goal from TaskPromptRL:

```bash
python scripts/send_navigation_goal.py --frame-name carry_box --offset 0.3
```

The inverted LiDAR mount is retained. Raw simulated Livox acceleration is in g,
matching the real driver. `T_lidar_imu` from the external GLIM configuration places
a separate simulated IMU site relative to the LiDAR; it is not approximated by
changing yaw. The camera/body kinematics are shared with real recognition.

GLIM should initialize from a stable robot. A fixed sensor startup delay is not a
readiness condition. Following a simulation clock reset, restart external GLIM
and submit a new goal; do not continue a trajectory across clock/map epochs.

## GLIM-free sim2sim

Use the same scene/camera/recognition, but omit external GLIM, pass
`--disable-lidar` to the simulator, and start deploy with:

```bash
python scripts/run_online_deploy.py --network lo --environment sim \
  --navigation-localization-source simulator
```

This explicitly uses MuJoCo truth from `rt/sportmodestate` for robot localization.
Recognition, target positions and marker search must not consume object truth.
`/evaluation/frame_pose_list` is only for evaluating results. There is no `auto`
selection or fallback from failed GLIM to truth.

## Real

The simulator is replaced by the existing Livox driver and camera bridge:

```bash
python scripts/camera_opencv.py <robot-interface> --no-display --no-publish-static-tf
python aruco/frame_pose_publisher.py --environment real --network <robot-interface>
```

Run the same external GLIM command/configuration. Start deploy with
`--environment real --navigation-localization-source glim --glim-config ...`
and the robot interface. `real + simulator` is rejected before creating robot
channels. This documentation is not a claim of hardware validation: real capture
timing, camera calibration and actual motion still require hardware checks.

## Domains and diagnostics

`--domain-id N` selects Unitree DDS on deploy, recognition and simulator. Keep
these equal. `ROS_DOMAIN_ID` independently selects ROS 2 communication for all
sensor consumers and GLIM. Nondefault domains are useful for isolated testing.
The legacy simulator `--domain_id` spelling is also accepted.

Recognition output is `/frame_pose_list` in `root_yaw`, with image timestamp and
clock-domain metadata. GLIM produces `/glim_ros/odom` for its IMU; the controller
converts that full pose into pelvis pose at the corresponding measured waist
state. See [frame and logging definitions](sensor_frames.md). Missing, stale or
mismatched frame input stops Navigation commands instead of selecting truth.
