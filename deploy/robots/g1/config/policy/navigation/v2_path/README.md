# Trajectory Navigation policy with corrected arm control

This configuration is for a Navigation policy retrained after the arm-up
tracking fix. Copy that 120-dimensional ONNX file to:

```text
v2_path/exported/policy.onnx
```

The wrist-pitch action scale in this directory includes the arm-up tracking
fix. Do not replace its ONNX file with a policy trained using the old scale.

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
