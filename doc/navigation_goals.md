# Explicit navigation goals and completion

Both sim and real use `GOAL x y yaw` in a frozen horizontal root frame at motion
start (+X forward, +Y left, positive yaw left; metres/radians). `GOAL_AT clock
stamp x y yaw` rebases an ArUco observation from image time to motion start using
root odometry history. No marker timeout triggers an automatic motion. Send an
explicit viewpoint goal, wait, then request a newly observed marker.

Backward goals use reverse Bezier tangents and signed velocity so that a pure
backward goal preserves heading. A goal specifies an endpoint, not a sequence
of backing up followed by turning. Use separate goals to enforce that order.
The path generator is not an obstacle avoidance planner.

## Local IPC protocol

- `STATUS` retains `READY`, `BUSY`, or `ERROR ...` for goal reception readiness.
- Accepted `GOAL` / `GOAL_AT` return `STARTED <id>`. This confirms consumption by
  Navigation, not arrival or successful localization/path initialization.
- `PROGRESS` returns `IDLE 0`, `RUNNING <id>`, `REACHED <id>`,
  `ERROR <id> <reason>`, or `CANCELLED <id> <reason>`.
- `REACHED` latches the first fresh localization sample inside both configured
  position/yaw tolerances. It is not a guarantee of settling or grasp success.
  Navigation continues its existing path tracking; no new standstill mode is
  enabled. Standstill behavior is deferred to a separate task.
- Timestamp/history failures, clock resets and rejected trajectories finish
  with `ERROR`. Temporary localization loss holds zero navigation command and
  remains running; there is no fallback to another localization source.
- Leaving Navigation before arrival cancels the running result. A replacement
  goal gets a different ID; the service retains only the newest result.
- The ID starts from the process's steady clock epoch and increments on each
  accepted goal. A waiting client must reject a different ID, including after
  controller restart. Delayed reports for an earlier ID cannot finish a new goal.
- `SEARCH` is removed; use explicit `GOAL` requests.

TaskPromptRL `send_navigation_goal.py --wait` checks the goal ID and fails on
non-success termination. Its arrival timeout does not cancel robot motion or
retry the request. Existing callers that compare a raw reply with exactly
`STARTED` must parse the first token and retain the new ID.

```bash
python scripts/send_navigation_goal.py --goal -0.15 0 0 --wait
python scripts/send_navigation_goal.py --goal 0 0 2.094395 --wait
python scripts/send_navigation_goal.py --frame-name place_table --offset 0.3 --wait
```

`goal_body_*`, `goal_yaw_error_rad` and `goal_position_error_m` log errors to the
final endpoint. `command_rel_*` and `position_error_m` remain path-reference errors.
Before this change, legacy `goal_body_*` and `goal_yaw_error_rad` duplicated the
path-reference errors and were misleading; do not interpret old logs as final
endpoint errors without recomputing from goal and robot poses.
