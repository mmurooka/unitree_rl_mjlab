# lift_walk_put v1 tracking policy

Final checkpoint: `model_30000.pt`, trained on the 100 randomized
`lift_walk_put` references generated on 2026-10-11.

- `exported/policy.onnx`: exported final policy.
- `params/deploy.yaml`: deployment configuration with joint observation
  offsets and action offsets matched to the training `params/env.yaml`.
- Policy period: 20 ms; input: 155 observations; output: 29 joint actions.
- Use this directory's deployment configuration together with this policy.
  The v0 configuration uses different default joint positions.

Policy SHA256:
`1295ef018d01dde08bc945550bca2befeac1ba84ff0086e551c91bd545666fbc`.
