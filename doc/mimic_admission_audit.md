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

See [command continuity](mimic_start_continuity.md) for runtime initialization and
[validation results](validation_results.md) for recorded simulation evidence.
