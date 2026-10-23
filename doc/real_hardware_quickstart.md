# Real hardware: manual transport with Viser recording

Run each block in a separate terminal, from the repository in its heading.
Assumes sibling checkouts, the interface below, and an already running Livox
stream. Complete the robot's low-level control preparation first.

## 1. GLIM — unitree_rl_mjlab, no venv

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch glim/launch/glim.launch.py
```

## 2. Camera — unitree_rl_mjlab, venv-tprl

```bash
source /opt/ros/jazzy/setup.bash
source ../TaskPromptRL/venv-tprl/bin/activate
python scripts/camera_opencv.py enx9c69d37c1711 --no-display \
  --calibration aruco/calibrations/d435i_243122071510_color_1920x1080.yaml
```

## 3. ArUco — unitree_rl_mjlab, venv-tprl

```bash
source /opt/ros/jazzy/setup.bash
source ../TaskPromptRL/venv-tprl/bin/activate
python aruco/frame_pose_publisher.py --environment real --network enx9c69d37c1711
```

## 4. Deploy — unitree_rl_mjlab, venv-mjlab

```bash
source /opt/ros/jazzy/setup.bash
source venv-mjlab/bin/activate
python scripts/run_online_deploy.py --network enx9c69d37c1711 --environment real \
  --navigation-localization-source glim
```

Enter Velocity with the robot remote before executing the sequence.

## 5. Viser + recording — TaskPromptRL, venv-tprl

Start deploy first: recording reads controller metadata at startup.

```bash
source /opt/ros/jazzy/setup.bash
source venv-tprl/bin/activate
python scripts/view_robot.py --live --environment real --glim --log-root ~/log
```

Open http://127.0.0.1:8080.

## 6. Manual sequence — TaskPromptRL, venv-tprl

```bash
source /opt/ros/jazzy/setup.bash
source venv-tprl/bin/activate
python scripts/run_transport.py --execute --manual --log-root ~/log
```

Review each yellow goal/path or Mimic final posture in Viser, then press Enter
in the sequence terminal. Ctrl+C stops further requests, not the active policy.

## Logs and replay

- `~/log/YYYYMMDD_HHMMSS_microseconds/sequence/`: task events, reviewed plan and reference files.
- `~/log/YYYYMMDD_HHMMSS_microseconds/viser/`: recorded viewer state for replay.
- `~/log/latest`: the latest session, managed automatically. Restart Viser for a new run.

After closing live Viser, replay the latest recording from TaskPromptRL:

```bash
source venv-tprl/bin/activate
python scripts/view_robot.py --replay ~/log/latest/viser
```
