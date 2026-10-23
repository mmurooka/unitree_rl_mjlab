# Reusable manual simulation checks

Use `scripts/sim_check.py` for repeatable, isolated step-by-step simulation runs.
This helper is loopback-only and does not launch GLIM. It prepares a **new empty**
run directory containing exact binary copies, configuration snapshots, separate
IPC endpoints, logs and a manifest of the selected source and allowed truth use.
It does not modify the normal manual-start configuration. The Python scripts and
policy assets remain linked by path to this checkout: preserve its commit and
avoid changing them during a run.

Activate the TaskPromptRL venv described in the [startup workflow](sim_real_workflow.md)
first; it must provide the ROS/SDK recognition dependencies. The examples below
explicitly select that active interpreter, so no `.venv-sensors` directory is needed.

Run from this repository root. GLIM mode uses the versioned `glim/config`
by default, as in the [normal startup workflow](../glim/README.md).

```bash
python scripts/sim_check.py prepare /path/to/new-run --localization simulator \
  --sensor-python "$(command -v python)"
# Or: --localization glim
```

In separate terminals, source the ROS environment and run:

```bash
python3 scripts/sim_check.py run /path/to/new-run simulator
python3 scripts/sim_check.py run /path/to/new-run controller
python3 scripts/sim_check.py run /path/to/new-run aruco
```

The helper's fallback recognition interpreter is `.venv-sensors/bin/python`;
`--sensor-python` above overrides it with your active venv. Only
controller/recognition/simulator run here;
the default TaskPromptRL sender converts in-process and sends directly to
`ipc:///path/to/new-run/deploy.sock`, so no motion worker is needed.

```bash
python3 scripts/sim_check.py joystick /path/to/new-run stand
# Wait for FixStand to settle, then:
python3 scripts/sim_check.py joystick /path/to/new-run velocity
# Confirm Velocity and a stable robot, then release hang:
python3 scripts/sim_check.py key /path/to/new-run 9
```

The hang toggle requires a logged Velocity state. The X11 key helper requires
one MuJoCo window; if more are open, operate the intended window manually.
The FIFO joystick interface only accepts the prepared FIFO, never a real device.

For GLIM checks, start external GLIM normally after the stable standing stage:

```bash
ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1 ros2 launch glim/launch/glim.launch.py
```

The manifest contains the exact ROS domain (default 84), DDS domain (default 83)
and IPC endpoints. From TaskPromptRL, use its sender with the same ROS domain:

```bash
ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1 python scripts/send_navigation_goal.py \
  --frame-name carry_box --offset .3 --endpoint ipc:///path/to/new-run/navigation.sock
```

Record observations and evaluation truth without feeding truth into control:

```bash
ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1 python scripts/record_frame_poses.py \
  /path/to/new-run/poses.jsonl --duration 30
```

Stop owned components with Ctrl+C in their terminals or:

```bash
python3 scripts/sim_check.py stop /path/to/new-run
```

The helper verifies saved PID start times before signaling. The launcher restores
terminal settings and kills its child process groups on Ctrl+C/SIGTERM, including
remaining descendants. Stop separately launched GLIM/recorders in their terminals.

For offline command/measurement comparison, use the SDK Python environment:

```bash
python scripts/record_robot_state.py /path/to/new-run/joints.jsonl \
  --environment sim --network lo --domain-id 83 --duration 180
```

This receiver publishes no commands. It records motor commands and measured
angles, velocities and body IMU orientation separately. Measured `tick_ms` is
simulation time in sim and robot boot time on real hardware; receipt timestamps
are monotonic wall time. Commands have receipt timestamps only. These timestamps
do not imply that a command and a measurement are simultaneous.

### Explicit one-time viewpoint calibration

The user authorized using object truth once to choose fixed relative waypoints
for this scene. `scripts/calibrate_sim_viewpoint.py` reconstructs a recorded
carrying pose from `poses.jsonl` and `joints.jsonl`, and renders candidate camera
views offline. It never sends goals or connects to the running simulator.

```bash
MUJOCO_GL=egl python scripts/calibrate_sim_viewpoint.py /path/to/recorded-run \
  /path/to/new-calibration --entry-stamp 139.5 --posture-stamp 150 \
  --back .9 1.2 1.5 --yaw .8 1.0 1.2
```

Use an active Python environment with MuJoCo 3.5 for this scene. Timestamps above are an
example from the recorded trial, not defaults for later runs. The tool uses
evaluation object poses and the fixed scene table pose to reconstruct the world
root pose. It freezes posture and box-to-root pose: visibility does not prove
collision clearance, stable carrying, or successful dynamic execution.
Save the chosen constants and their calibration provenance, then validate them
with the normal image-based pipeline. Future transport must use those frozen
constants without reading object truth. This exception does not permit truth
input to marker recognition or camera transforms.

For an offline odometry comparison, use `scripts/evaluate_navigation_frames.py`:

```bash
python scripts/evaluate_navigation_frames.py /path/to/run/poses.jsonl \
  /path/to/run/01_box_navigation_pose.csv /path/to/run/05_table_navigation_pose.csv \
  --anchor-world 1 0 0 --output /path/to/run/frame_evaluation.json
```

The anchor is the fixed pickup-table origin in this scene (world x/y/yaw).
This is evaluation truth only. Timestamp-matched root poses are aligned once
in XY and yaw at the first sample, preserving subsequent drift and directional
errors. The output reports missing samples and rejects mixed source files.
These errors describe ideal simulated sensors, not real-world calibration.

For post-sequence box placement, explicitly evaluate the labeled truth recording
without passing it back to control:

```bash
python scripts/evaluate_box_placement.py /path/to/run/poses.jsonl \
  /path/to/run/sequence/events.jsonl --table-top .56 \
  --table-half .2 .275 --box-half .13 .175 .095 \
  --output /path/to/run/placement_evaluation.json
```

These dimensions belong to the supplied scene. The tool requires a completed
sequence and a post-completion window, then checks box center over the table,
bottom near its top, low tilt and stable position. This is finite geometric
evaluation, not contact-force sensing or proof of reliability over many trials.
