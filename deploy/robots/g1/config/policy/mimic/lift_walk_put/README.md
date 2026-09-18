# lift_walk_put tracking policy

Source training run: `logs/rsl_rl/g1_tracking/2026-09-16_23-43-59`.

- `exported/policy.onnx` is a copy of the run's `policy.onnx`.
- `params/deploy.yaml` was prepared from the run's `params/env.yaml`, using
  the existing dance deployment configuration as a template and checking
  the observation order against the run's ONNX metadata.
- Policy period: 20 ms; input: 155 observations including `motion_phase`;
  output: 29 joint actions.

The deployment files are self-contained and do not link to the training logs.
