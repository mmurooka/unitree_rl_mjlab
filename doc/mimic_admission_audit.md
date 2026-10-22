# Reproduce an OnlineMimic admission decision offline

Set `FSM.OnlineMimic.log_admission_trace: true` in an isolated run's controller
configuration. The default is false. This adds one JSON log record per admission
attempt with the exact observation tensors passed to ONNX, previously issued
commands, candidate processed commands, reference path and admission result.
It does not change policy input, thresholds or motor commands. Both command arrays
are SDK order in radians. Do not confuse them with measured or reference angles.

With `G1_BUILD_ONLINE_TESTS=ON`, the build also produces:

```bash
deploy/robots/g1/build/replay_mimic_admission \
  deploy/robots/g1/config/policy/mimic/v1_lift_walk_put \
  /path/to/controller.log > /path/to/admission_replay.jsonl
```

This executable opens no robot interface. It reruns the recorded observation and
requires its processed output to match the trace within 1e-4 rad before analysis.
It then compares two alternatives for only the `last_action` observation: zero-reset
initialization and the previous command expressed through this policy's scale/offset.
The result is an offline sensitivity comparison, not a proposed automatic retry
or evidence that the alternate initialization is physically stable. It supports
the current single-input, 29-joint policy with unit observation scales and history
length one; incompatible configurations are rejected.

Preserve the effective configuration and ONNX hash with the run. Standard
admission and actual physical playback remain the authority on execution.

## Correction and validation (2026-10-23)

OnlineMimic now initializes `last_action` from the actual preceding processed
command, converted through the candidate policy's inverse affine action mapping.
Measured angles are not used for this history. Resetting to zero incorrectly told
the policy that its nominal-offset command preceded the new task. Admission still
checks the resulting first output against the actual preceding command with the
unchanged 20-degree threshold, and reuses that checked output on acceptance.

A recorded rejected put reproduced exactly offline: zero history gave 24.14 degrees,
while previous-command history gave 9.37 degrees. Full image-based transport then
completed with simulator localization (lift 8.04 / put 11.92 degrees) and external
GLIM (7.97 / 17.03 degrees). The GLIM put observation with zero history would give
31.13 degrees. Both runs placed the box stably on the destination table in offline
evaluation; no threshold relaxation, automatic retries or new training were used.
These are finite simulated trials, not a real-hardware or reliability guarantee.
