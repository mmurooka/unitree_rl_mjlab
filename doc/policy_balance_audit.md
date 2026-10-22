# Velocity / OnlineMimic audit without new training

Audit date: 2026-10-23. Selected deployed artifacts are Velocity `velocity/v0`
and OnlineMimic `mimic/v1_lift_walk_put`. No rewards, observations, gains or trained
weights were changed in this audit. New recovery training is explicitly deferred.

Both deploy configurations use SDK joint order 0..28, 29 position outputs and a
20 ms policy period. Nominal joint positions and action offsets agree. Maximum
absolute proportional-gain difference is 0.0494 (0.347%); derivative-gain difference
is 0.0422 (2.99%). These differences are consistent with rounded Velocity values,
not evidence of a large gain discrepancy explaining directional falls. Action
scales are similar but are not identical, especially the small wrist scales.

Velocity observes angular velocity, projected gravity, velocity commands, gait
phase, joint offsets/velocities and previous action. Mimic observes reference joint
positions/velocities, relative torso orientation, angular velocity, joint offsets/
velocities, previous action and motion phase. Neither deployed actor directly
observes base linear velocity. Mimic is a feedback policy, not open-loop playback.
Its relative torso orientation provides tilt information despite not exposing a
separate projected-gravity term.

Current training source differs in its objectives: Velocity emphasizes velocity
tracking and gait; tracking emphasizes reference body pose/velocity and contact.
Tracking's `foot_slip_penalty` masks foot velocity by **reference contact**, so a
recovery step during reference double support can be penalized. Contact matching
can also compete with stepping. These are plausible mechanisms, not causal proof
for the deployed model. Both current source configurations include horizontal
velocity perturbations of ±0.5 m/s; tracking's interval is 1–3 s versus Velocity's
5–6 s. Thus lack of perturbation training cannot be assumed. Play configurations
remove automatic pushes.

OnlineMimic runs the policy indefinitely at the last reference with zero reference
joint velocity. Current training resamples at motion end; exact long-hold and
perturbation coverage for this deployed checkpoint has not been established.
The deployed ONNX hashes and available historical training exports do not uniquely
establish a corresponding complete training setup. Do not label the current source
reward table as the proven training configuration of both deployed policies.

Non-training corrections already made separately include command-to-command
admission (instead of comparing command to measurement), keeping the source state
on rejection, and preserving fitted measured initial posture in reference generation.
These correct implementation issues; they do not demonstrate Velocity-like push
recovery. Long carrying holds have lost the box in observed transport runs, which
also cautions against assuming indefinite hold is a stable manipulation mode.

Remaining policy work, deferred with new training: establish exact checkpoint/data
provenance, evaluate matched force/impulse directions and phases, separate playback
from final hold, then assess contact/foot-motion rewards and recovery reference
coverage. Retraining or replacing the standing policy is outside the present scope.
