# Read-only controller visualization snapshots

Both existing local ZMQ REP endpoints accept `VIEW` and return JSON (`schema: 1`).
This command never starts/queues motion, changes modes, or changes progress.
It is available independently of whether the controller accepts motion requests.
The viewer polls at a bounded rate; it uses no motor-command publisher.

Navigation VIEW includes `root`, `path`, `goal_id`, `progress`, `reason`, `status`.
`root` is the same selected root localization used by Navigation, even while another
policy is active: source (`glim` or explicitly enabled `simulator`), source stamp,
clock domain, XYZ position, wxyz quaternion, and wall age since the last *new* stamp.
GLIM data has already been converted from sensor IMU to pelvis with measured FK.
The simulator source is robot truth and remains forbidden on real hardware.
`path` stores the actually adopted trajectory samples `[x,y,yaw]`, step_dt, goal ID,
and fixed world anchor `[x,y,z,yaw]` at its source timestamp. It is cleared when a
new goal is accepted, so an earlier path is not mislabeled with the new goal ID.
The last path can remain visible after a state switch; check progress/status.

OnlineMimic VIEW includes status, numeric mode (inactive=0, velocity=1,
navigation=2, mimic=3), measured state and last playback. Measured state includes
SDK-order q/dq, pelvis IMU wxyz, tick_ms and stamp. Sim uses its tick clock. Real
`ros_receipt_approx` uses query-time system clock: it is approximate, not a hardware
capture timestamp. Stale/missing SDK state is null. Consumers must keep this
uncertainty distinct from geometric error.

Playback includes a monotonically assigned process-local ID, exact loaded NPZ
path, frame count, fixed fps=50, actual policy frame, and root localization captured
at admission. A controller restart starts a new ID sequence. The final frame remains
reported while held. A local viewer should copy/hash the NPZ into its recording;
if the sender disabled debug retention the file may already be gone. Its absence
must be reported rather than displaying a different reference.

Absolute reference translation is not a policy tracking command. A reference model
placed at the measured start root is a visualization alignment. The actual policy
output can deviate; the reference is not a prediction of contact or task success.
The simulator does not generate extra truth transforms for these snapshots.

`METADATA` on the motion endpoint is also read-only. It returns effective controller
YAML, selected source/environment/domain, project/executable paths, sensor-model
path and external GLIM directory for recording identity. It does not load a motion.
Playback VIEW includes `stamp` and `clock_domain` sampled when the policy frame is
published. Sim uses the current SDK simulation tick; real uses software wall time.
A viewer can select a reported frame bracketed by root history and interpolate
measured posture/root to that stamp, rather than overlaying newest inputs with
unequal latency. Real hardware capture synchronization is still approximate.

`PLAN x y yaw` on the Navigation endpoint returns a passive root_yaw reference
curve (51 samples, schema 1, `poses`, `tangent_scale`). It uses the same configured
Bezier geometry as actual Navigation, without changing mode, progress, or goal
queues. It has no time parameterization and is not a collision/physics prediction.
`VIEW.path` remains the actual time-parameterized path adopted after execution.
