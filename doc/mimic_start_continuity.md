# OnlineMimic start continuity

At a motion boundary the controller compares the previous **motor position
command** with the candidate policy’s first **motor position command**, in SDK
joint order. The existing `start_joint_threshold_degrees` setting (20 degrees by
default) limits that change. It does not compare measured angles with commands
or reference angles: compliant, low-gain tracking normally has such errors.

MotionPrompt supplies the reference; the compared commands are policy outputs
after action scaling/offset, not reference joint angles or raw action tensors.
The candidate `last_action` is initialized from the previous motor command
through the inverse candidate action mapping.

The candidate is evaluated from current measured state in a reusable second
inference context, without publishing its output. On acceptance that context
becomes active and the already computed first action is used once. On rejection,
the previous held motion and its observation/action history remain intact.
Velocity/Navigation admission runs before the source exits: rejection leaves
its policy history, trajectory, goal ID and arm target running unchanged. A
rejected image-based Navigation goal is not restarted at a new origin/time.
No extra process is started. This checks command continuity, not equal torques across
policies with different gains, nor disturbance-recovery performance.

For measured initial-pose fitting and repeated support-plane checks, see
[TaskPromptRL current-pose semantics](https://github.com/isri-aist/TaskPromptRL/blob/main/doc/current_pose_validation.md).
[Validation results](validation_results.md) summarize the bounded simulation checks.

The optional `G1_BUILD_ONLINE_TESTS` CMake target `test_online_motion` checks
pre-exit rejection and prepared-action playback using isolated domain 232 and
test-only topics; it never publishes motor commands. Its 180-degree fixture
threshold is deliberately test-only. Actual sim/real deployment retains the
configured 20-degree limit and must be validated separately.
