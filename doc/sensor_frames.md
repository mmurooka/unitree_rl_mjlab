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
This foundation does not by itself remove the old TF-dependent runtime path.
