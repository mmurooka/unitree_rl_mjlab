# G1 trajectory Navigation policy

Use this 120-input policy with the `params/deploy.yaml` in this directory.
Observation semantics, joint order and action scales must match training; input
width alone does not establish compatibility. The active controller selects
this directory in `config/config.yaml`.

See the repository [startup workflow](../../../../../../../doc/sim_real_workflow.md),
[goal protocol](../../../../../../../doc/navigation_goals.md) and
[telemetry](../../../../../../../doc/navigation_telemetry.md) for runtime use.

Arm controls in Navigation are:

- `L1 + Up`: the configured basic raised pose
- `L1 + Down`: the lowered pose
- `L1 + Right`: the next randomly shuffled pose from `arm_pose_eval.npz`

The evaluation library uses a seed different from training and contains an
equal mix of symmetric and asymmetric random poses. A random pose is always
approached from the lowered pose; switching between random poses also returns
through the lowered pose first.
