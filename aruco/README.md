# ArUco object poses (same recognition in sim and real)

`frame_pose_publisher.py` publishes `/frame_pose_list` from camera images,
CameraInfo, measured SDK motor angles and the pelvis IMU quaternion. Its
`root_yaw` frame has the pelvis origin, horizontal robot-forward +X, left +Y,
and up +Z. Waist motion and body tilt are included through model FK.
Recognition does not use GLIM, ROS TF, or simulated world poses.

Use Python 3.10 with ROS Humble (or Python matching your installed ROS), NumPy,
PyYAML, OpenCV with ArUco, and `unitree_sdk2_python`. Reuse the TaskPromptRL venv
from the [startup workflow](../doc/sim_real_workflow.md#common-terminal-setup),
including its online dependencies. Install the official Unitree SDK2 Python package there
as for `scripts/camera_opencv.py`; DDS may require `CYCLONEDDS_HOME=/usr/local`.
Do not use an unrelated Python version with the ROS binary modules.

After starting the simulator:

```bash
source /opt/ros/humble/setup.bash
python aruco/frame_pose_publisher.py --environment sim --network lo
```

For a robot, start the camera bridge separately, then the same recognition node:

```bash
python scripts/camera_opencv.py <robot-interface> --no-display \
  --calibration /path/to/actual_640x480_calibration.yaml
python aruco/frame_pose_publisher.py --environment real --network <robot-interface>
```

`--domain-id` selects Unitree DDS (default 0); it is separate from ROS_DOMAIN_ID.
`--glim` is a compatibility alias with no effect on recognition.
The camera topics and marker map are in `simulate/config.yaml`; the same map is
used for both environments. Toggle `frame_pose_list.aruco.display` for preview.
Press `q` to close only the preview. Regenerate marker textures using
`python aruco/generate_markers.py` when marker IDs change.

Fixed waist/sensor geometry is `config/g1_sensor_frames.yaml`, exported from the
MJCF; see [sensor frames](../doc/sensor_frames.md). The model optical frame is
used for both `camera_color_optical_frame` and `g1_front_camera_optical_frame`.
A different physical camera requires its calibrated model transform. CameraInfo
must match the image frame and dimensions. Actual calibration accuracy is still
to be validated on hardware; a syntactically valid K does not establish accuracy.

State is interpolated at the image timestamp, rather than using the latest TF.
In sim, SDK tick and camera stamp both use simulation seconds. In real, SDK state
is timestamped on receipt in ROS time; the camera bridge also timestamps received
images. This is a documented approximation to capture time, not proof of hardware
synchronization. Stale state, mismatched timestamps and gaps suppress poses.

JSON includes `stamp`, `reference_frame: root_yaw`, `source: aruco`,
`clock_domain` (`simulation` or `ros`), and `frame_pose_list` entries with
`frame_name` and `rel_pose: [x,y,z,qx,qy,qz,qw]`. The reference is the robot at
that stamp, not the robot at a later goal start time.

The simulator's `/evaluation/frame_pose_list` is evaluation-only object truth.
It must never be consumed for navigation goals or target search, even in the
GLIM-free localization mode. The simulator publishes no world-derived camera TF. Setting `frame_pose_list.source: ground_truth` is rejected.

`object_to_marker` specifies marker pose in the object frame; its inverse turns
the observed marker pose into an object pose. Marker size is the black square's
outer edge (12 cm in the supplied configuration), excluding the quiet zone.

Tests: `python -m unittest discover -s aruco -p test_frame_pose_publisher.py`
and `python -m unittest discover -s tests -p test_robot_frames.py`.

The real bridge requires a 640×480 stream and an explicitly supplied actual
calibration. It has no default intrinsic values and applies no resize/scaling.
`--save-on-exit frame.png` saves the valid image and calibration/stream metadata.
See [real camera setup](../doc/camera_calibration_audit.md) for acquisition limits
and calibration requirements. Static TF is off by default.
