# Navigation logs and plots

From the repository root, start the plotter before or after entering Navigation:

```bash
python3 scripts/plot_deploy_navigation.py
```

Each Navigation entry truncates and then streams
`deploy/robots/g1/log/navigation_pose.csv` at the policy rate. The CSV contains
the closest sampled path point's relative pose and cumulative path progress,
velocity commands, raw policy actions, commanded arm pose targets, encoder
joints, joint targets actually written to LowCmd,
projected gravity, and the localization position/quaternion. The
`slam_pose_*` columns contain the converted pelvis pose in the selected
localization frame. `raw_odom_*` contains the original sensor pose. Use `source`
(`glim` or `simulator`) to distinguish estimated localization from robot truth;
sim supports either source. See [sensor frames](sensor_frames.md).
Quaternion columns are ordered `qx,qy,qz,qw`. `command_joint_15` through
`command_joint_28` are the 14 arm-pose target angles. They start from the
measured pose captured immediately before Navigation and can then be changed
with the deploy `L1` controls. They are intentionally not the policy's processed
full-body output. `action_joint_*` is the last exact `q` value written to the
mapped LowCmd motor entry. Encoder/action columns remain available for all 29
joints. Unavailable data is written as `nan` without stopping the other
telemetry.

The complete sampled reference is written separately to
`deploy/robots/g1/log/navigation_target_path.csv` as soon as localization is
available. This includes the whole future path rather than only reference
points near the robot's current progress.

The plotter opens a pose window and a joint window. The pose window shows the
relative error to the closest sampled path point, plots `closest_progress_m`
over time, and overlays the measured robot trajectory with the dashed green
target path and final goal. In the joint window,
`command_joints` shows the
14 commanded arm joints, while `encoder_joints` and `action_joints` show all 29
G1 joints. To filter the encoder/action plots, pass joint indices; the command
plot then shows only arm joints contained in that selection. For example:

```bash
python3 scripts/plot_deploy_navigation.py --joints 15 18 22 25
```

The title reports whether the data source is `simulator` or `glim`; missing or
stale localization is shown explicitly. This file-based interface is identical
for both simulator and GLIM localization, and keeps GUI
work outside the control process.

The log paths, enable flag, and flush interval can be changed with
`pose_log_path`, `target_path_log_path`, `pose_log_enabled`, and
`pose_log_flush_interval` in the Navigation `deploy.yaml`. If required, install plotting dependencies with
`python3 -m pip install matplotlib numpy`.
