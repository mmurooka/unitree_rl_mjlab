# Trajectory Navigation policy with corrected arm control

This configuration is for a Navigation policy retrained after the arm-up
tracking fix. Copy that 120-dimensional ONNX file to:

```text
v2_path/exported/policy.onnx
```

The wrist-pitch action scale in this directory includes the arm-up tracking
fix. Do not replace its ONNX file with a policy trained using the old scale.

Navigation goals are supplied through `send_navigation_goal.py` as `x y yaw`.
They are interpreted in the robot frame captured when the goal starts: +x is
forward, +y is left, and yaw is in radians. A goal sent from Velocity or a
finished/holding OnlineMimic automatically enters Navigation. A goal sent while
Navigation is active restarts the trajectory from the current pose. The arm
command is initialized from the measured arm angles immediately before entry,
so a lifted pose is retained without distinguishing pickup/place iterations.

The generated target path is written to `log/navigation_target_path.csv` and
the measured robot path to `log/navigation_pose.csv`.
Running `scripts/plot_deploy_navigation.py` also opens a per-joint arm tracking
window that overlays the selected target and encoder measurement and reports
the latest RMS and maximum absolute errors.

Arm controls in Navigation are:

- `L1 + Up`: the basic raised pose used previously
- `L1 + Down`: the lowered pose
- `L1 + Right`: the next randomly shuffled pose from `arm_pose_eval.npz`

The evaluation library uses a seed different from training and contains an
equal mix of symmetric and asymmetric random poses. A random pose is always
approached from the lowered pose; switching between random poses also returns
through the lowered pose first.
