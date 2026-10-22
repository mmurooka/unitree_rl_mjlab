# OnlineMimic start continuity

At a motion boundary the controller compares the previous **motor position
command** with the candidate policy’s first **motor position command**, in SDK
joint order. The existing `start_joint_threshold_degrees` setting (20 degrees by
default) limits that change. It does not compare measured angles with commands
or reference angles: compliant, low-gain tracking normally has such errors.

The candidate is evaluated from current measured state in a reusable second
inference context, without publishing its output. On acceptance that context
becomes active and the already computed first action is used once. On rejection,
the previous held motion and its observation/action history remain intact.
Velocity/Navigation entry rejection returns to the source policy. No extra
process is started. This checks command continuity, not equal torques across
policies with different gains, nor disturbance-recovery performance.

`--current-pose` still queries measured joint angles. The sender fits them to the
same fixed, horizontal foot contacts for every generation. Fits exceeding 2 mm
position or 0.01 rad orientation residual are rejected instead of silently
continuing with a drifting support plane. Repeated noisy-ankle fits are tested;
real repeated manipulation and push recovery still require separate validation.
