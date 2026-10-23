# G1 simulation validation

Snapshot: 2026-10-23. These are bounded tests of the supplied policies and scene,
not a real-hardware validation or a reliability estimate. Runtime contracts are
in [command continuity](mimic_start_continuity.md) and
[sensor frames](sensor_frames.md); reusable evaluation commands are in
[simulation checks](sim_check.md).

## Short retreat and matched approach clearance (2026-10-23)

These results precede the operator-requested change to a 90-degree turn.

The tested plan used backward 0.4 m, left turn 0.85 rad, and destination-table
center offset 0.43 m. Both table approaches target 0.23 m pelvis clearance from
the near edge. The command limits are 30 degrees for legs and 20 for waist/arms.
Offline viewpoint evaluation used authorized scene truth once, with measured
joints/IMU and the recorded 960x540 camera resolution. A 1.5-rad turn occluded the
marker behind the carried box; 0.8–0.9 rad exposed it. Runtime used fixed constants
and image recognition, without object truth in control.

Both GLIM-free and external-GLIM runs completed all Navigation goals and lift/put.
Five-second placement evaluation passed in each: final table-relative box centers
were (-0.0074, 0.0006, 0.6521) m and (-0.0624, 0.0379, 0.6521) m respectively.
No GLIM timing warning occurred. These single runs do not establish a success rate.
All owned processes were stopped; no real hardware was commanded.

The initial run directories contain poses.jsonl, joints.jsonl, sequence events,
references and placement evaluations, but no Viser replay recording. For direct
Viser replay, run a read-only live viewer with --record alongside the sequence;
ordinary audit logs are not the viewer recording format. A subsequent GLIM-only
run with Viser recording also completed transport and passed five-second
placement evaluation (box center -0.0566, 0.0019, 0.6521 m in the table frame).
Its viewer_record directory was opened in offline replay, with playback controls
verified in a browser. Camera images and point clouds are live-only.

## Current workflow recheck (2026-10-23)

After rebuilding the current simulator and controller, the documented startup
order was checked with loopback-only DDS domain 83 / ROS domain 84, isolated
binary/configuration copies, and FIFO Xbox input via `sim_check`. FixStand and
Velocity preceded hang release. GLIM was launched externally from this checkout's
`glim/launch/glim.launch.py` after standing stabilized. This tests the equivalent
components and ordering, not physical Xbox hardware.

- GLIM: all four Navigation goals and lift/put completed. Offline placement
  evaluation passed over five seconds; final box center in the destination-table
  frame was (0.0603, -0.0263, 0.6525) m.
- GLIM-free: all four Navigation goals and lift completed. Put was rejected at
  SDK joint 6 (right hip pitch): 20.89 degrees between the preceding Navigation
  position command and the candidate Mimic position command, above the unchanged
  20-degree limit. This run did **not** complete transport.
- The reported `points=0` GLIM warning was not reproduced. A 12-second audit
  received 120 clouds and 2502 IMU messages with no timestamp rewinds. One cloud's
  first point timestamp was 30204009999.99 ns, corresponding to 30.20401 s,
  rather than zero. No GLIM timing warnings appeared during the transport run.

A separate GLIM-free repeat with admission tracing enabled stopped because the
destination marker was not received within 10 seconds after the turn. It did
not reach put, so it did not reproduce the 20.89-degree rejection. The recorded
lift output matched offline ONNX replay exactly (maximum command change 10.88
degrees). This is not evidence that put admission or repeated transport is fixed.

Object truth was used only for offline placement evaluation. GLIM-free control
additionally used explicit robot-localization truth. No real robot channels were
used; all test-owned processes were stopped. These are bounded local runs, not
proof that another PC's binary/configuration/stream combination is correct.

## Earlier image-based box transport

Both runs completed approach, lift, fixed backward/turn goals, destination-marker
approach and put with the standard 20-degree command-continuity limit. No output
blending, automatic retry, threshold relaxation or new training was used.

| Localization | Lift / put command change | Final box center in destination-table frame (m) |
|---|---|---|
| Simulator robot truth | 8.04° / 11.92° | (0.0602, -0.0033, 0.6521) |
| External GLIM | 7.97° / 17.03° | (0.0257, -0.0493, 0.6521) |

Offline evaluation found placement stable for about five seconds after sequence
completion. Box bottom was about 2.94 mm below the nominal table surface under
MuJoCo soft contact. Evaluation object truth was recorded separately and was
not fed back to control. GLIM-free control used robot-localization truth explicitly;
recognition in both modes used images, measured joints and IMU with model FK.

GLIM odometry versus evaluation root truth, aligned once at the first sample,
had XY RMSE 1.04 mm and yaw RMSE 0.000724 rad over 130 matched samples. Ideal
simulated sensors do not establish real sensor calibration or timing accuracy.

## Admission replay

The exact ONNX observation from a rejected put reproduced its output offline.
Changing only previous-action initialization from zero to the actual preceding
command gave 24.14° versus 9.37°. For the final GLIM put observation, the accepted
history gave 17.03°; a zero-history counterfactual gave 31.13°. These comparisons
support the implemented history initialization, not a retry strategy. Use the
[admission replay tool](mimic_admission_audit.md) with exact policy/config hashes.

## Unloaded measured-pose repetitions

Three lift/put pairs (six motions) completed. The first lift used nominal posture;
the next five used measured current pose. Admission changes were 5.89°, 5.48°,
3.43°, 6.13°, 3.66°, 6.30°. At segment ends, maximum relative sole-point height
difference was 0.040 mm and maximum tilt was 0.277°. Absolute sole-point heights
were 4.32–4.41 mm, using recorded root truth for offline evaluation only.

Relative support geometry uses measured joints and IMU; absolute height additionally
uses localization. The geometric sole point differs from collision surfaces, so
these values do not imply an air gap. No cumulative deterioration was seen over
these three unloaded pairs with approximately one-second intervals. Loaded,
long-hold, disturbance and hardware performance remain separate questions.

## Evidence retention

The local, non-Git report collection records the run names
`2026-10-23_handoff_history`, `2026-10-23_handoff_history_glim`, and
`2026-10-23_mimic_repeat_history`. Those recordings are not repository fixtures.
To reproduce the checks, preserve the run manifest, effective YAML, ONNX hashes,
controller logs, sequence events, pose/state recordings and exact converted motions.
