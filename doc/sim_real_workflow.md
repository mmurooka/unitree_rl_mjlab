# Individual G1 startup: simulation and real hardware

This is the main startup procedure for Xbox-operated simulation and separate
sensor/controller terminals. Build/install first using [setup](setup_en.md) and
[TaskPromptRL online setup](https://github.com/isri-aist/TaskPromptRL/blob/main/doc/online_motion_prompt_quickstart.md).
For isolated per-run binaries, domains and recordings, use [sim_check](sim_check.md)
instead. Neither workflow starts GLIM for you.

| Mode | Sensor processes | Localization | Startup section |
|---|---|---|---|
| Sim with GLIM | MuJoCo camera/LiDAR + ArUco | External GLIM | [Sim with GLIM](#sim2sim-with-glim) |
| Sim without GLIM | MuJoCo camera + ArUco | Explicit simulator robot truth | [GLIM-free sim](#glim-free-sim2sim) |
| Real | Real camera + Livox driver + ArUco | External GLIM | [Real hardware](#real-hardware) |

## What can be grouped

There is no single command that starts the entire sensor/GLIM/controller/viewer
stack. The following tools group only the indicated work:

| Tool | Scope |
|---|---|
| `run_online_deploy.py` | Starts g1_ctrl and manages its shutdown. Optional `--with-motion-worker` also starts the compatibility worker. |
| `sim_check.py prepare`, then separate `run` commands | Prepares isolated sim configuration, binaries, endpoints and logs; simulator/controller/ArUco are still started in separate terminals. GLIM is external. |
| TaskPromptRL `run_transport.py --execute` | Executes the whole reviewed task after startup; it does not launch the sensor/control stack. |
| TaskPromptRL `scripts/run_transport_interactive.sh` | Runs individual task commands with terminal pauses after startup. |

For sim, prefer the numbered manual procedure below when operating the physical
Xbox. Use [sim_check](sim_check.md) for repeatable scripted button/recording checks.
For real, start sensors and controller separately; the task sequencer can then
run a plan adapted to the real layout.

## Before startup

Replace all `/path/to/...` placeholders with your actual checkout locations.
Install the TaskPromptRL `.[online]` dependencies in its Python 3.10 venv using
the [online setup](https://github.com/isri-aist/TaskPromptRL/blob/main/doc/online_motion_prompt_quickstart.md#1-prepare-the-environment).
Each terminal below includes its own ROS, directory and Python environment setup.

After pulling simulator changes, rebuild before starting these terminals:

```bash
source /opt/ros/humble/setup.bash
cd /path/to/unitree_rl_mjlab
cmake -S simulate -B simulate/build
cmake --build simulate/build -j
```

Updating source/configuration alone does not update the simulator executable.
The shared GLIM configuration expects absolute per-point timestamps in nanoseconds;
older simulator binaries emitted relative seconds. Restart simulator, GLIM and
deploy together after updating; do not continue a GLIM session across a sim reset.

Deploy and the GLIM launch file default to this checkout's `glim/config`.
The simulator uses fixed MJCF sensor geometry and never reads GLIM settings.
See [GLIM setup](../glim/README.md) for installation and custom configuration.

Unitree DDS and ROS domains are independent; both default to 0. If you customize
them, keep each domain consistent across its participating processes.
Run only one simulator and controller for these endpoints.

## Sim2sim with GLIM

Check `simulate/config.yaml`: `use_joystick: 1`, `joystick_type: xbox`, and the
actual `joystick_device` (default `/dev/input/js0`). Connect the Xbox gamepad.
Each numbered command below runs in its own terminal using the setup shown.

### 1. Simulator

```bash
source /opt/ros/humble/setup.bash
cd /path/to/unitree_rl_mjlab
simulate/build/unitree_mujoco --network lo \
  --scene src/assets/robots/unitree_g1/xmls/scene_g1_table_box_glim.xml
```

MuJoCo publishes the simulated camera, LiDAR and IMUs. Do not start the real camera
bridge in sim. Room walls are hidden in the MuJoCo viewer by default; sensor rays
and collisions still include them. See [simulator settings](../simulate/README.md).

### 2. Controller and gamepad

```bash
source /opt/ros/humble/setup.bash
cd /path/to/unitree_rl_mjlab
python3 scripts/run_online_deploy.py --network lo --environment sim \
  --navigation-localization-source glim
```

The launcher starts only g1_ctrl by default. Motion conversion runs in TaskPromptRL.

1. Press **LT + D-pad Up** to enter FixStand; wait for its posture transition.
2. Press **RT + A** to enter Velocity; center the sticks.
3. Confirm Velocity is active in the controller log, then focus the MuJoCo window
   and press keyboard **9** to release hang. Wait for a stable stance.

Always enter Velocity before releasing hang. `LT + B` enters Passive;
`RT + A` returns Navigation/OnlineMimic to Velocity. Simulated Xbox input is
translated into Unitree controller fields by MuJoCo.

### 3. External GLIM

Initialize GLIM from the stable robot:

```bash
source /opt/ros/humble/setup.bash
cd /path/to/unitree_rl_mjlab
ros2 launch glim/launch/glim.launch.py
```

The inverted LiDAR mount and separate IMU offset are fixed in MJCF. GLIM
calibration describes that hardware geometry independently. Raw Livox acceleration is in g; GLIM applies the configured scale.
A fixed startup delay is not a readiness condition. After a simulation clock
reset, restart GLIM and submit a new goal rather than continuing across epochs.

### 4. Marker recognition

```bash
source /opt/ros/humble/setup.bash
cd /path/to/unitree_rl_mjlab
source /path/to/TaskPromptRL/venv-tprl/bin/activate
python aruco/frame_pose_publisher.py --environment sim --network lo
```

Recognition uses images, measured joints and pelvis IMU with model kinematics,
independent of GLIM and TF. Wait for current localization and the target marker
before issuing commands. See [frames and timestamp rules](sensor_frames.md).

### 5. Optional Viser display

From TaskPromptRL, use read-only viewing alongside individual commands:

```bash
source /opt/ros/humble/setup.bash
cd /path/to/TaskPromptRL
source venv-tprl/bin/activate
python scripts/view_robot.py --live --environment sim --read-only --glim
```

Open `http://127.0.0.1:8080`. For browser preview followed by its execution button,
omit `--read-only` and use the browser as the command owner. See
[Viser](https://github.com/isri-aist/TaskPromptRL/blob/main/doc/viser_viewer.md)
for recording/replay. `--glim` enables GLIM point-cloud display; visibility can
also be toggled in the browser. The viewer does not start sensors, GLIM or g1_ctrl.

### 6. Individual commands or transport sequence

From TaskPromptRL, approach one observed marker:

```bash
source /opt/ros/humble/setup.bash
cd /path/to/TaskPromptRL
source venv-tprl/bin/activate
python scripts/send_navigation_goal.py --frame-name carry_box --offset 0.3 --wait
```

Marker-based commands require three consecutive image observations with maximum
pairwise differences of 5 cm and 10 degrees in fixed localization coordinates.
The sender compares timestamp-matched controller root poses and averages the
accepted observations. There is no minimum duration. Missing/skipped images reset
the count; duplicates do not count again. Keep both repositories updated because
the ArUco publisher supplies `image_sequence`. Thresholds are in TaskPromptRL's
`config/marker_stability.yaml`; the same checks apply to Viser and the sequencer.

For the whole supplied scene, choose **one** workflow:

- `bash scripts/run_transport_interactive.sh`: individual commands using Navigation arrival-or-Enter and Motion Enter waits.
  It starts with the box-approach goal; do not separately repeat that step.
- Manual sequence with Viser: follow [the complete two-terminal procedure below](#manual-transport-with-viser-previews).
- `python scripts/run_transport.py --execute --run-dir /path/to/new-sequence-run`: run the reviewed plan automatically.
- Viser browser: preview and explicitly execute each command.

The scene-specific plan uses separate backward **0.4 m** and left-turn **0.85 rad**
goals before observing the destination marker. Those constants were selected once
with offline evaluation truth; runtime uses the fixed constants and fresh images.
Do not apply this plan unchanged to a different layout. MotionPrompt box dimensions
and pickup/place references are also plan inputs. See the
[sequence guide](https://github.com/isri-aist/TaskPromptRL/blob/main/doc/transport_sequence.md).

Motion sending exits on acceptance by default; `--wait` waits for Enter only. Confirm the controller
reports `FINISHED; holding final reference, READY` before the next manual stage.
Navigation `--wait` ends on goal-specific arrival or Enter; neither indicates stationary feet.
Enter ends CLI waiting without stopping motion. There are no separate shell reads.

### Manual transport with Viser previews

Use this procedure after starting the sensors, recognition, localization and
controller, and entering Velocity. In simulation, release hang only after Velocity
is active. The commands below enable GLIM point-cloud display. For GLIM-free
simulation, omit `--glim`. On real hardware, change the viewer's
`--environment sim` to `--environment real` and keep `--glim`.

Use the viewer command below **instead of** the optional viewer in step 5.
Start g1_ctrl first. Use the same log root in both terminals; Viser creates the
timestamped run directory and enables recording automatically.

**Terminal A — start the passive viewer:**

```bash
source /opt/ros/humble/setup.bash
cd /path/to/TaskPromptRL
source venv-tprl/bin/activate
python scripts/view_robot.py --live --environment sim --glim --log-root /path/to/logs
```

Open `http://127.0.0.1:8080`. `--log-root` enables read-only sequence viewing
and recording; the browser has no execution button for the sequence.

**Terminal B — prepare, review and execute each step:**

```bash
source /opt/ros/humble/setup.bash
cd /path/to/TaskPromptRL
source venv-tprl/bin/activate
python scripts/run_transport.py --execute --manual --log-root /path/to/logs
```

Each viewer start creates a new timestamped session; restart Viser for a new
run. `sequence/` holds task logs/references and `viser/` holds viewer recordings.
Before each terminal prompt, Viser shows:

| Step | Preview before Enter |
|---|---|
| Navigation | Yellow goal position/orientation and reference curve. |
| Mimic | Yellow robot at the reference's final posture, overlaid with the blue measured robot. |

Inspect the preview, then press **Enter in Terminal B** to execute that step.
Preparation sends no motion request. The sequencer waits for Navigation arrival
or Mimic playback completion before preparing the next step. Mimic executes the
same converted file shown in the preview, without regeneration. Explicit relative
Navigation goals remain relative to the execution start; marker-derived goals
stay fixed in localization coordinates. The displayed curve/posture describes a
reference, not guaranteed physical tracking or collision clearance.

Ctrl+C in Terminal B stops further sequence commands; it does not stop the robot's
active policy. Completion or cancellation clears the pending preview. Run logs,
reference files and the reviewed plan are saved under the run directory.
Replay the most recent viewer recording with
`python scripts/view_robot.py --replay /path/to/logs/latest/viser`.

## GLIM-free sim2sim

Replace steps 1 and 2 with these commands and omit step 3:

```bash
source /opt/ros/humble/setup.bash
cd /path/to/unitree_rl_mjlab
simulate/build/unitree_mujoco --network lo --disable-lidar \
  --scene src/assets/robots/unitree_g1/xmls/scene_g1_table_box_glim.xml
```

```bash
source /opt/ros/humble/setup.bash
cd /path/to/unitree_rl_mjlab
python3 scripts/run_online_deploy.py --network lo --environment sim \
  --navigation-localization-source simulator
```

Omit `--glim` from the Viser command in step 5 or the manual-preview procedure.
For example, passive viewing without GLIM is:

```bash
python scripts/view_robot.py --live --environment sim --read-only
```

Keep `--log-root /path/to/logs` when using manual previews and recording.
All other steps are identical. This source explicitly uses MuJoCo robot truth
from `rt/sportmodestate` for localization/displacement and goal time compensation.
Recognition, camera transforms and marker goals do not use object truth.
`/evaluation/frame_pose_list` is evaluation-only. There is no automatic source
selection or fallback when GLIM fails.

## Real hardware

For the checked PC/interface and venv setup, use the concise
[copy-and-paste startup with timestamped Viser recording](real_hardware_quickstart.md).

Start the external Livox driver without RViz in its own terminal:

```bash
source /opt/ros/humble/setup.bash
source /path/to/ws_livox/install/setup.bash
cd /path/to/unitree_rl_mjlab
ros2 launch glim/launch/livox.launch.py
```

This uses the installed driver's MID360_config.json. Configure its host/sensor
IPs as described in [GLIM setup](../glim/README.md); an optional
`user_config_path:=/path/to/MID360_config.json` selects another driver config.
Do not also launch `rviz_MID360_launch.py` or a second Livox driver.

Replace MuJoCo with that driver and the real camera stream. Use the same
external GLIM command/configuration. Complete the robot's established low-level
control preparation before starting g1_ctrl; the simulator's keyboard hang toggle
does not apply. Mode buttons use the existing Unitree remote. This workflow does
not forward a PC-connected Xbox directly to a real robot.

The existing G1 preparation procedure is to power on while suspended, wait for
zero-torque mode, then use `L2 + R2` to enter debug/damping mode. Connect Ethernet
and configure the PC interface for the robot network (the existing example is
`192.168.123.222/24`; choose a free host address on the configured robot subnet). Confirm
the robot's low-level control preparation using its established operating procedure.
Once g1_ctrl is ready, use the remote's `L2 + Up` for FixStand and `R2 + A` for
Velocity; physical support release follows the real-robot procedure.

Set the actual robot-facing network interface in each real-hardware terminal.
Find its name with `ip -brief link`; names vary between PCs. For example, replace
`YOUR_ROBOT_INTERFACE` below with the selected interface name:

```bash
export ROBOT_INTERFACE=YOUR_ROBOT_INTERFACE
```

Start recognition and deploy in separate terminals:

```bash
source /opt/ros/humble/setup.bash
cd /path/to/unitree_rl_mjlab
source /path/to/TaskPromptRL/venv-tprl/bin/activate
python aruco/frame_pose_publisher.py --environment real --network "$ROBOT_INTERFACE"
```

```bash
source /opt/ros/humble/setup.bash
cd /path/to/unitree_rl_mjlab
python3 scripts/run_online_deploy.py --network "$ROBOT_INTERFACE" --environment real \
  --navigation-localization-source glim
```

For the checked robot camera (D435i serial **243122071510**), use the tracked
calibration file shown below. For another camera, supply its own calibration for
the native received image. The PC resizes the complete 16:9 image to 960×540 with matching CameraInfo. Start it
in another terminal:

```bash
source /opt/ros/humble/setup.bash
cd /path/to/unitree_rl_mjlab
source /path/to/TaskPromptRL/venv-tprl/bin/activate
python scripts/camera_opencv.py "$ROBOT_INTERFACE" --no-display \
  --calibration aruco/calibrations/d435i_243122071510_color_1920x1080.yaml
```

The SDK cannot select resolution or retrieve intrinsics. The
[camera setup](camera_calibration_audit.md) explains the required upstream
native calibration and local image conversion. There is no unverified
fallback. Viser uses `--environment real --glim`, including when viewing manual
sequence previews.
`real + simulator` is rejected before creating robot channels. Real camera
calibration, capture timing and physical motion still require hardware validation.

## Stop

In sim, switch to Passive when ready to stop, then Ctrl+C the controller launcher
and other terminals, and close MuJoCo. The launcher stops its own controller/worker
process group; GLIM and independently launched programs must be stopped separately.
Ctrl+C in a sender, sequencer or viewer stops further requests but does not cancel
an already accepted motion. Inspect the active controller state before resuming.
