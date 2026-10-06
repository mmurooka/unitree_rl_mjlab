# Trajectory Navigation policy with corrected arm control

This configuration is for a Navigation policy retrained after the arm-up
tracking fix. Copy that 120-dimensional ONNX file to:

```text
v2_path/exported/policy.onnx
```

The wrist-pitch action scale in this directory includes the arm-up tracking
fix. Do not replace its ONNX file with a policy trained using the old scale.

On the first Navigation entry, deploy snapshots the `carry_box` pose from
`rt/navigation/target_pose` and approaches its front. When an OnlineMimic
motion entered from Navigation finishes, the next Navigation entry switches to
the empty `place_table` pose from `rt/navigation/place_target_pose` and retains
the motion's final arm pose while walking there. The current scene/config
produces pickup goal `(0.62, 0.0, 0.0)` and place-table goal approximately
`(0.2, 0.62, pi/2)` in MuJoCo world coordinates. Stand-off distances and
approach-face yaw offsets are configurable independently.

On the real robot, `target_source: auto` selects fixed goals when GLIM is the
active localization source. The first Navigation entry goes 1 m forward with
no yaw change. After the lift motion, the next entry goes 1 m left and finishes
at +90 degrees, while retaining the lifted arm pose. Both offsets are relative
to the robot pose captured on that Navigation entry and can be changed with
`fixed_carry_box_goal` and `fixed_place_table_goal`. Set `target_source` to
`fixed` or `simulator` to override automatic selection.

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
