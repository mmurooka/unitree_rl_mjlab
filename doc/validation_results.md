# G1 simulation validation

Snapshot: 2026-10-23. These are bounded tests of the supplied policies and scene,
not a real-hardware validation or a reliability estimate. Runtime contracts are
in [command continuity](mimic_start_continuity.md) and
[sensor frames](sensor_frames.md); reusable evaluation commands are in
[simulation checks](sim_check.md).

## Image-based box transport

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
