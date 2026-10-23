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
| TaskPromptRL `command.sh` | Runs individual task commands with terminal pauses after startup. |

For sim, prefer the numbered manual procedure below when operating the physical
Xbox. Use [sim_check](sim_check.md) for repeatable scripted button/recording checks.
For real, start sensors and controller separately; the task sequencer can then
run a plan adapted to the real layout.

## Common terminal setup

Examples use ROS Humble, sibling repositories under `~/src` and the shared Python
3.10 environment. Run this setup in each terminal, adjusting the external GLIM
path to the **same complete configuration directory used on real hardware**:

```bash
source /opt/ros/humble/setup.bash
source ~/src/TaskPromptRL/.venv-online-ros/bin/activate
export ROS_DOMAIN_ID=0
export G1_GLIM_CONFIG="$HOME/src/glim/config"
cd ~/src/unitree_rl_mjlab
```

The simulator/controller read that directory to match sensor frames and extrinsics;
they do not write it. `SLAM/GLIM_config` is a ROS-settings reference, not the active
complete GLIM installation. Unitree DDS `--domain-id` defaults to 0; keep it equal
on simulator, controller and recognition. ROS_DOMAIN_ID is independent and must
match all ROS sensor consumers and GLIM. Run only one controller for these endpoints.

## Sim2sim with GLIM

Check `simulate/config.yaml`: `use_joystick: 1`, `joystick_type: xbox`, and the
actual `joystick_device` (default `/dev/input/js0`). Connect the Xbox gamepad.
Each numbered command below runs in its own terminal after the common setup.

### 1. Simulator

```bash
simulate/build/unitree_mujoco --network lo --glim-config "$G1_GLIM_CONFIG" \
  --scene src/assets/robots/unitree_g1/xmls/scene_g1_table_box_glim.xml
```

MuJoCo publishes the simulated camera, LiDAR and IMUs. Do not start the real camera
bridge in sim. Room walls are hidden in the MuJoCo viewer by default; sensor rays
and collisions still include them. See [simulator settings](../simulate/README.md).

### 2. Controller and gamepad

```bash
python scripts/run_online_deploy.py --network lo --environment sim \
  --navigation-localization-source glim --glim-config "$G1_GLIM_CONFIG"
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
ros2 run glim_ros glim_rosnode --ros-args -p config_path:="$G1_GLIM_CONFIG"
```

The inverted LiDAR mount and separate IMU offset follow the model and external
`T_lidar_imu`. Raw Livox acceleration is in g; GLIM applies the configured scale.
A fixed startup delay is not a readiness condition. After a simulation clock
reset, restart GLIM and submit a new goal rather than continuing across epochs.

### 4. Marker recognition

```bash
python aruco/frame_pose_publisher.py --environment sim --network lo
```

Recognition uses images, measured joints and pelvis IMU with model kinematics,
independent of GLIM and TF. Wait for current localization and the target marker
before issuing commands. See [frames and timestamp rules](sensor_frames.md).

### 5. Optional Viser display

From TaskPromptRL, use read-only viewing alongside individual commands:

```bash
cd ~/src/TaskPromptRL
python scripts/view_robot.py --live --environment sim --read-only
```

Open `http://127.0.0.1:8080`. For browser preview followed by its execution button,
omit `--read-only` and use the browser as the command owner. See
[Viser](https://github.com/isri-aist/TaskPromptRL/blob/main/doc/viser_viewer.md)
for recording/replay. The viewer does not start sensors or g1_ctrl.

### 6. Individual commands or transport sequence

From TaskPromptRL, approach one observed marker:

```bash
cd ~/src/TaskPromptRL
python scripts/send_navigation_goal.py --frame-name carry_box --offset 0.3 --wait
```

For the whole supplied scene, choose **one** workflow:

- `bash command.sh`: individual commands with terminal pauses between stages.
  It starts with the box-approach goal; do not separately repeat that step.
- `python scripts/run_transport.py --execute --manual`: confirm each plan step.
- `python scripts/run_transport.py --execute`: run the reviewed plan automatically.
- Viser browser: preview and explicitly execute each command.

The scene-specific plan uses separate backward **1.2 m** and left-turn **1.0 rad**
goals before observing the destination marker. Those constants were selected once
with offline evaluation truth; runtime uses the fixed constants and fresh images.
Do not apply this plan unchanged to a different layout. MotionPrompt box dimensions
and pickup/place references are also plan inputs. See the
[sequence guide](https://github.com/isri-aist/TaskPromptRL/blob/main/doc/transport_sequence.md).

Motion sending exits on acceptance, not playback completion. Confirm the controller
reports `FINISHED; holding final reference, READY` before the next manual stage.
Navigation `--wait` reports goal-specific arrival, not stationary feet.

## GLIM-free sim2sim

Replace steps 1 and 2 with these commands and omit step 3:

```bash
simulate/build/unitree_mujoco --network lo --disable-lidar \
  --scene src/assets/robots/unitree_g1/xmls/scene_g1_table_box_glim.xml
```

```bash
python scripts/run_online_deploy.py --network lo --environment sim \
  --navigation-localization-source simulator
```

All other steps are identical. This source explicitly uses MuJoCo robot truth
from `rt/sportmodestate` for localization/displacement and goal time compensation.
Recognition, camera transforms and marker goals do not use object truth.
`/evaluation/frame_pose_list` is evaluation-only. There is no automatic source
selection or fallback when GLIM fails.

## Real hardware

Replace MuJoCo with the existing real Livox driver and camera stream. Use the same
external GLIM command/configuration. Complete the robot's established low-level
control preparation before starting g1_ctrl; the simulator's keyboard hang toggle
does not apply. Mode buttons use the existing Unitree remote. This workflow does
not forward a PC-connected Xbox directly to a real robot.

The existing G1 preparation procedure is to power on while suspended, wait for
zero-torque mode, then use `L2 + R2` to enter debug/damping mode. Connect Ethernet
and configure the PC interface for the robot network (the existing example is
`192.168.123.222/24`). Select the actual interface in all commands below. Confirm
the robot's low-level control preparation using its established operating procedure.
Once g1_ctrl is ready, use the remote's `L2 + Up` for FixStand and `R2 + A` for
Velocity; physical support release follows the real-robot procedure.

Start recognition and deploy in separate terminals (replace the interface):

```bash
python aruco/frame_pose_publisher.py --environment real --network enx9c69d37c1711
```

```bash
python scripts/run_online_deploy.py --network enx9c69d37c1711 --environment real \
  --navigation-localization-source glim --glim-config "$G1_GLIM_CONFIG"
```

For the real camera, follow the [camera setup](camera_calibration_audit.md) and use
calibration matching the delivered image. Viser uses `--environment real`.
`real + simulator` is rejected before creating robot channels. Real camera
calibration, capture timing and physical motion still require hardware validation.

## Stop

In sim, switch to Passive when ready to stop, then Ctrl+C the controller launcher
and other terminals, and close MuJoCo. The launcher stops its own controller/worker
process group; GLIM and independently launched programs must be stopped separately.
Ctrl+C in a sender, sequencer or viewer stops further requests but does not cancel
an already accepted motion. Inspect the active controller state before resuming.
