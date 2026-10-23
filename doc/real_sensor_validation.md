# Real and simulated sensor validation

The camera and Livox/GLIM pipelines were checked on 2026-10-23 without starting
a robot controller or issuing actuator commands. See
[workflow](sim_real_workflow.md) for per-terminal startup and
[camera calibration](camera_calibration_audit.md) for the serial-specific profile.

## Shared contracts

| Stream | Sim | Real |
| --- | --- | --- |
| Color image | 960×540, ideal MJCF camera | Native JPEG resized to 960×540, full 16:9 frame |
| CameraInfo | Derived from MJCF fovy | Device factory calibration transformed with the image |
| Marker poses | Image + measured joints/IMU + model FK | Same recognition code and FK |
| Annotated camera | Recognition JPEG with original image header | Same; available in Viser beside 3D |
| Livox PointCloud2 timestamp | Absolute simulation time, float64 nanoseconds | Driver absolute time, float64 nanoseconds |
| Livox acceleration | g units | g units |
| GLIM | External GLIM with repository config | Same config and external GLIM |

The color FOV is approximately 70.5° × 43.4° in both modes. Sim uses an ideal
centered principal point and square pixels; its K is not identical to the real
camera's factory K. Real resizing does not reduce robot-to-PC JPEG traffic.
GLIM converts acceleration using 9.80665 and point timestamps using 1e-9.
The simulator uses fixed sensor sites, not GLIM configuration, to generate data.
No simulator world-pose camera transform is used by recognition.

## Passive inspection

After starting the selected camera, recognition and external Livox/GLIM processes:

```bash
source /opt/ros/humble/setup.bash
source /path/to/TaskPromptRL/venv-tprl/bin/activate
cd /path/to/unitree_rl_mjlab
python scripts/inspect_sensors.py --environment real --network YOUR_ROBOT_INTERFACE   --duration 15 --output /path/to/sensor_report.json
# For simulation use --environment sim --network lo.
```

This tool subscribes only. It does not start drivers, control policies or motors.
The network argument enables a measured LowState subscription for the gravity
comparison; omit it to inspect only ROS streams. Use the same ROS domain as the
sensor processes. Reports include publisher counts, stream dimensions, rates,
timestamp rewinds, point time ranges and static gravity consistency.

## Observed results and limits

- Live real and isolated sim each delivered approximately 10 Hz point clouds,
  200 Hz IMU and 10 Hz GLIM odometry, without timestamp rewinds in these short runs.
  Odometry used parent `odom` and child `livox_imu`; no self-transform error.
- Both cameras delivered 960×540 images. Real ID 0 (12 cm) was detected; Viser's
  read-only browser displayed the annotated camera without a controller or GLIM.
  A stale receipt is visibly labeled. Camera images are currently live-only,
  not stored in Viser replay recordings.
- **LiDAR mounting remains unresolved:** measured static acceleration differs by
  about 11.7° from gravity predicted using pelvis IMU, measured waist joints and
  the current model mount. The comparison already includes measured body tilt.
  Measured leg FK predicts sole tilts of about 0.8° and 1.5°, consistent with the
  operator's report that both soles contact level ground. Do not assume that the
  discrepancy is explained by the robot leaning. A single static pose cannot
  determine yaw or a complete mounting transform, so no guessed correction was
  applied. Confirm the physical sensor-axis mounting before relying on real
  Navigation direction.
- The real camera and LowState use approximate PC receipt timestamps. The robot's
  onboard clock was not synchronized; it was not changed. Hardware capture
  synchronization, delivery latency and camera extrinsics need further validation.
- Intrinsics came from the connected device, but optical calibration accuracy still
  needs a multi-pose board check. Marker recognition alone does not certify robot
  relative coordinates or safe motion.
- No real walking, manipulation or policy transition was tested. The isolated sim
  check held the robot suspended and tested sensors/GLIM only. Simulator truth was
  not used for recognition or target construction.
