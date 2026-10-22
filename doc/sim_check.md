# Reusable manual simulation checks

Use `scripts/sim_check.py` to preserve the previous step-by-step workflow in Git.
This helper is loopback-only and does not launch GLIM. It prepares a **new empty**
run directory containing exact binary copies, configuration snapshots, separate
IPC endpoints, logs and a manifest of the selected source and allowed truth use.
It does not modify the normal manual-start configuration. The Python scripts and
policy assets remain linked by path to this checkout: preserve its commit and
avoid changing them during a run.

```bash
python3 scripts/sim_check.py prepare /path/to/new-run --localization simulator
# Or: --localization glim --glim-config /path/to/external/glim/config
```

In separate terminals, source the ROS environment and run:

```bash
python3 scripts/sim_check.py run /path/to/new-run simulator
python3 scripts/sim_check.py run /path/to/new-run controller
python3 scripts/sim_check.py run /path/to/new-run aruco
```

The default recognition interpreter is `.venv-sensors/bin/python`; override with
`prepare --sensor-python ...`. Only controller/recognition/simulator run here;
start the existing motion worker separately until the direct sender is available.

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
ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1 ros2 run glim_ros glim_rosnode \
  --ros-args -p config_path:=/path/to/external/glim/config
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
