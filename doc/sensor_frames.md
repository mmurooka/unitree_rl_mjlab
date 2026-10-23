# Common sensor frames

`config/g1_sensor_frames.yaml` contains fixed model geometry exported from the G1
MJCF by `python scripts/export_sensor_frames.py`. It is not a GLIM configuration.
Runtime consumers need NumPy/PyYAML, not MuJoCo. Regenerate it after changing the
robot's waist chain or sensor mounts. The export records the source and SHA256;
the regression test checks it against the source model.

`T_A_B` maps coordinates in B into A. Measured motor angles use the SDK's 29-motor
order, and the body IMU quaternion uses SDK `w,x,y,z` order. The physical root is
`pelvis`. `root_yaw` has the same origin, +Z up and +X along the horizontal
projection of pelvis +X. It is independent of the SLAM map's origin and yaw.

Recognition uses `T_horizontal_root * FK(q_measured, camera_optical)`. It must not
read simulated world poses or SLAM poses. The exported optical transform includes
the MuJoCo camera-to-ROS optical-axis conversion. The inverted LiDAR mount is a
full 3D rotation in the model, not an added yaw correction.

For localization, `T_odom_root = T_odom_imu * inverse(T_root_lidar(q) * T_lidar_imu)`.
GLIM's `odom` topic estimates the Livox IMU pose. Its IMU extrinsic comes from the
externally selected GLIM configuration; it is distinct from the pelvis IMU and
from the sensor's mount on the torso. Waist motion must be evaluated at the
odometry timestamp, before displacement is calculated.

`StateHistory` interpolates measured angles and quaternion at image/odometry
stamps. Inputs must share a clock domain. It rejects stale data and sampling gaps,
and discards the previous epoch on time rewind. Hardware body-state reception
timestamps are an approximation to measurement time; capture/transport delays
must be documented when assessing real-robot accuracy.

Validation: `python -m unittest discover -s tests -p test_robot_frames.py`.
The tests use MuJoCo world poses only as an independent evaluation oracle, never
as input to the production transform. They cover random root/waist attitudes,
map-yaw invariance, inverted mounts, IMU offsets, stale data and clock resets.
ArUco recognition calls this function directly and does not require ROS TF.

## Navigation runtime

The controller requires `--navigation-localization-source glim|simulator`.
`--environment real` is the default; `simulator` is rejected for real hardware.
For GLIM pass `--glim-config /absolute/path/to/config`, the same directory passed
to the externally launched `glim_rosnode`. It is read-only. Its `config.json`
selects the actual sensor/ROS files; nonstandard file names and JSON comments
are supported. `--sensor-model` optionally selects another exported model.
`--domain-id` selects Unitree DDS; it does not change ROS_DOMAIN_ID.

The GLIM subscriber checks parent/child names and samples measured waist angles
in a bounded history at the odometry timestamp. Both message age and receipt age
must be fresh. Repeated or reordered odometry cannot refresh its lifetime.
Simulation clock rewind invalidates navigation: restart external GLIM and submit
a new goal; a running trajectory must not cross clock/map epochs. Ground-truth
localization subscribes only to `rt/sportmodestate` and still uses the same root
origin/heading convention. It needs neither GLIM nor a ROS-enabled controller.

In Navigation CSV logs, `slam_pose_*` columns contain the **converted
pelvis pose**, in the localization world frame. `raw_odom_*` retains the received
GLIM IMU pose (or truth root pose for `simulator`). `localization_stamp_s` is the
sample time and `localization_age_s` its age; `robot_*` is the displacement since
start in the frozen horizontal root frame. The `source` column distinguishes
estimated from ground-truth localization. Ground truth never supplies marker
positions or camera transforms.

## Image time and Navigation start time

`GOAL x y yaw` is expressed in the frozen start-root frame. Marker observations
use `GOAL_AT simulation|ros timestamp x y yaw`. Root poses bracket the image time;
position and shortest-path yaw are interpolated (no extrapolation, no gaps above
0.3 s). The image-local goal is transformed into the start-root frame before
trajectory generation. Missing history waits with zero commands for up to 2 s,
then rejects that goal; a new observation is required. `STARTED` acknowledges
policy acceptance, not localization readiness or goal arrival.

The GLIM-free DDS robot-localization sample includes position, root quaternion
and simulation time in one `rt/sportmodestate` message. This is an explicitly
permitted truth input, used only for robot displacement and the above time
compensation; object/camera truth is never read by these consumers. Both modes
keep a 5 s root-pose history even outside Navigation. Clock rewind clears history
and invalidates the active goal. Real image/state time remains an approximation
based on receipt time; hardware capture latency still needs measurement.
