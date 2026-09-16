# Online MotionPrompt (G1 29 DoF)

The TaskPromptRL sender transmits one existing MotionPrompt NPZ per ZMQ request.
No robot, joint order, FPS, or motion ID is added to the payload. Input/output
are 50 Hz. The default transport is local ZMQ IPC, not a custom TCP protocol.

## Setup

Install `libzmq3-dev` (C++ headers/library) and install this repository's updated
Python dependencies (`pip install -e .`) in the existing mjlab environment.
Build with your usual ROS environment if navigation is enabled, or disable
ROS navigation for a standalone build:

```sh
cmake -S deploy/robots/g1 -B deploy/robots/g1/build -DG1_NAVIGATION_WITH_ROS2=OFF
cmake --build deploy/robots/g1/build -j2
```

Set `FSM.OnlineMimic.policy_dir` in `deploy/robots/g1/config/config.yaml` to the
tracking policy you already use offline. The example inherits the existing
`Mimic_walk` path `config/policy/mimic/walk_1000/`; provide that policy or change
the path if it is absent. The policy stays fixed across requests. The initial
implementation checks 29 joints in the current converter's motor order and a
20 ms policy period. It does not adapt models to new motions.

```sh
python scripts/run_online_deploy.py --network lo
```

This starts the converter once and starts the G1 controller. `lo` is for local
simulation; use your existing DDS interface for other deployments. Either child
exiting stops the other. The converter can also be launched separately with
`python scripts/serve_motion_prompt.py`.

Enter FixStand, then Velocity through the normal controls. **R1 + Y** enters
OnlineMimic; **R2 + A** returns to ordinary Velocity. OnlineMimic waits using the
velocity policy with x/y/yaw commands clamped to zero, including joystick
commands. Passive transitions remain available. Sending while inactive returns
an error and does not change the controller's FSM state.

Generate/send using TaskPromptRL's `scripts/send_motion_prompt.py`. Its default
interactive mode reuses the initialized IK object; Enter generates a motion
using the command-line parameters, and `q` quits. `--once` sends immediately;
`--input-file FILE.npz` sends an existing 50 Hz file once.

## Playback contract

- The Python receiver validates the existing NPZ and uses a persistent
  `MotionPromptConverter`, preserving offline interpolation, velocity
  calculation, and all-body FK. The model is initialized once.
- The worker stores the converted NPZ and notifies the C++ receiver over
  `ipc:///tmp/task_prompt_rl_deploy.sock`. These two processes must share a
  filesystem. C++ reads and validates the file outside the control loop.
- Immediately before starting, every reference joint angle is compared to the
  corresponding measured angle. A difference **greater than 30 degrees** rejects
  the request with the joint index, difference, and threshold. It does not stop
  the controller process. Configure `start_joint_threshold_degrees` locally.
- Every accepted motion reinitializes the existing heading correction and
  observation/action histories, and starts at frame zero. Absolute position is
  not a tracking observation. Root orientation still contributes to the
  relative torso orientation observation.
- Each converted frame runs for 20 ms. After the last frame's interval, the
  last position/orientation is held, reference joint velocity is zero, and the
  final phase is retained for `end_hold_seconds` (default 1.0). Waiting then
  resumes with zero velocity. Motions should end in a standing pose.
- `motion_phase` has one float32 element, `frame / num_frames`, not seconds.
  The final value is `(num_frames - 1) / num_frames`, strictly below 1. Only
  policies configured with this observation consume it; online mode does not
  alter the policy input dimension.
- Conversion, playback, and holding do not accept a replacement or queue.
  Requests receive `BUSY`. Automatic retry is disabled. `STARTED` acknowledges
  acceptance by the control loop, not completion of the physical motion.
  A reply timeout leaves the execution outcome unknown; inspect deploy before
  manually submitting again. After full reception, playback does not depend on
  continued sender connectivity.

The converter deliberately retains the existing endpoint-exclusive sampling:
for example, 904 input frames at 50 Hz currently produce 903 output frames.
Playback duration is defined by the **converted** frame count.

## Debugging

The sender saves exact transmitted bytes to `/tmp/task_prompt_rl/sent/`.
The receiver saves each received payload to
`/tmp/task_prompt_rl/received/motion_*/motion_prompt.npz`, including BUSY/rejected
requests, and successful conversions to `policy_input.npz` beside it.
These are unique local filenames, not IDs transmitted in the NPZ.

Both sides accept `--debug-dir` and `--no-save-debug`. The launcher forwards these
options to the receiver. With retention disabled, conversion files are kept
only until deploy finishes loading them, then deleted. `/tmp` files are useful
for offline debugging but may disappear on reboot; retained files accumulate
until explicitly cleaned up.

```sh
python scripts/convert_motion_prompt_npz.py --robot g1 \
  --input-file /tmp/task_prompt_rl/received/motion_EXAMPLE/motion_prompt.npz \
  --input-fps 50 --output-fps 50 --device cpu \
  --output-path /tmp/reconverted_policy_input.npz
```

For disconnected senders, `--timeout-seconds` bounds waiting and no request is
resent. ZMQ sockets are owned by their respective threads; the receiver uses a
ROUTER socket to reject additional requests while conversion is in progress.
See the [PyZMQ API documentation](https://pyzmq.readthedocs.io/en/stable/api/zmq.html)
for the socket lifecycle and timeout options.

## Tests

```sh
python -m unittest discover -s tests -p test_motion_service.py
cmake -S deploy/robots/g1 -B deploy/robots/g1/build \
  -DG1_NAVIGATION_WITH_ROS2=OFF -DG1_BUILD_ONLINE_TESTS=ON
cmake --build deploy/robots/g1/build -j2
deploy/robots/g1/build/test_online_motion "$PWD/deploy/robots/g1" /tmp/short_policy_input.npz
```

The C++ smoke test uses the bundled dance policy only to exercise inference and
the online state machine with a frozen synthetic robot state. Supply a short
converted NPZ (e.g. 10 frames). It uses DDS domain 232 on loopback, a test-only
LowState topic, and never constructs a motor-command publisher. It is not a
test of physical tracking performance or of your selected deployment policy.
