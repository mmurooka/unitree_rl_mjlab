# ArUco frame pose publisher

`frame_pose_publisher.py` detects the marker IDs configured under
`frame_pose_list.aruco.markers` in `simulate/config.yaml`, converts each marker
pose to its associated object pose, and publishes the existing
`std_msgs/msg/String` JSON contract on `/frame_pose_list`.

## Simulation

Set the source in `simulate/config.yaml`:

```yaml
frame_pose_list:
  source: "aruco"
```

Start the simulator, then run:

```bash
source .venv-ar/bin/activate
python aruco/frame_pose_publisher.py
```

With `frame_pose_list.aruco.display: true`, this also opens the robot camera
view annotated with detected marker borders, IDs, frame names and pose axes.
Press `q` while that window is focused to close the view without stopping pose
publication. Set `display: false` for a headless run.

Set `source: "ground_truth"` to return to the MuJoCo body-pose publisher. In
ArUco mode, the simulator publishes truth separately on
`/frame_pose_list_ground_truth`; `send_navigation_goal.py` uses it only to turn
toward a requested marker that is initially outside the camera view. The final
approach always uses the marker pose from `/frame_pose_list`. The simulator
also broadcasts the moving `imu_yaw -> camera_link` transform needed by the
ArUco node.

Marker search backs up 0.15 m while turning by default. On a real robot, run
the following from the TaskPromptRL repository and pass a fixed search
direction relative to the robot's current heading. This prevents access to any
simulation-truth topic:

```bash
python scripts/send_navigation_goal.py \
  --frame-name carry_box \
  --search-yaw 1.57 \
  --search-backoff 0.15
```

When `--search-yaw` is omitted, simulation search continues to obtain the
direction from `/frame_pose_list_ground_truth`.

The simulated scene uses ID 0 on the robot-facing side of `carry_box`, ID 1 on
top of `pickup_table`, and ID 2 on top of `place_table`. Regenerate their PNGs
after changing the dictionary or IDs with:

```bash
python3 aruco/generate_markers.py
```

## Real robot

`scripts/camera_opencv.py` bridges the JPEG stream returned by Unitree SDK2 to
the same ROS 2 interface used in simulation:

- `/camera/camera/color/image_raw` (`sensor_msgs/msg/Image`, `bgr8`)
- `/camera/camera/color/camera_info` (`sensor_msgs/msg/CameraInfo`)

In one terminal, source ROS 2 and an environment containing
`unitree_sdk2py`, then pass the Ethernet interface connected to the robot:

```bash
source /opt/ros/jazzy/setup.bash
source .venv-ar/bin/activate
python scripts/camera_opencv.py <robot-network-interface>
```

The OpenCV preview remains enabled by default; press `Esc` or `q` to stop the
node. Add `--no-display` when running headless. The topic names already match
`simulate/config.yaml`, so start pose detection in another ROS-sourced
terminal with:

```bash
python aruco/frame_pose_publisher.py --glim
```

The camera bridge loads `aruco/g1_front_camera.yaml` by default and scales its
intrinsics if the SDK returns a resolution other than 1920x1080. Those initial
values come from the existing G1 ArUco setup; replace that file with calibration
from the particular robot for accurate metric poses. A different standard ROS
camera-calibration YAML can be selected with `--calibration`.

The same YAML also contains the initial
`livox_frame -> g1_front_camera_optical_frame` camera extrinsic. The bridge
publishes it on `/tf_static`, connecting the camera to GLIM's observed
`odom -> livox_frame` transform. These extrinsics are derived from the G1
simulation model and must be replaced with measured values for precise
real-robot approaches. If another node already publishes a calibrated camera
transform, start the bridge with `--no-publish-static-tf` instead.

`--glim` makes the ArUco node consume that connected TF tree. It removes the
current GLIM base yaw and origin while preserving roll/pitch and the camera
mounting transform, producing marker poses in the same current robot-relative
convention expected when Navigation captures its initial GLIM pose. It labels
the JSON reference as `glim_base_yaw`. The defaults match this project's GLIM
output (`odom -> livox_frame`); override them when needed with
`--glim-world-frame` and `--glim-base-frame`.

Use the same marker YAML mapping with `source: "aruco"`. Without `--glim`, the
simulation behavior remains unchanged and uses the configured `reference_frame`.

If the camera driver has no TF, set `use_tf: false` and enter the calibrated
camera pose as `static_reference_to_camera` in ROS pose order
`[x,y,z,qx,qy,qz,qw]`. A fixed transform is suitable only when the camera is
rigid relative to the chosen reference frame.

Before running the complete deploy sequence, verify the bridge independently:

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic echo /camera/camera/color/camera_info --once
ros2 run tf2_ros tf2_echo odom g1_front_camera_optical_frame
```

`object_to_marker` is the marker coordinate frame's pose in the object's frame.
It accounts for a marker being mounted on a side or top surface, so downstream
code still receives the object's center pose rather than the marker pose.
`marker_size` is the black marker's outer-edge length, excluding its white quiet
zone. For the supplied images, print the 12 cm marker on a 15 cm white square
and mount it with the orientation described by `object_to_marker`.
