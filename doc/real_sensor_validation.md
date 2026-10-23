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
- **LiDAR mounting remains unresolved:** before the nominal axis correction, measured static
  acceleration differed by about 11.7° from model-predicted gravity in the first
  test pose. The corrected nominal mount leaves 6.04° in the later recorded pose
  (details below). The comparison already includes measured body tilt.
  Measured leg FK predicts sole tilts of about 0.8° and 1.5°, consistent with the
  operator's report that both soles contact level ground. Do not assume that the
  discrepancy is explained by the robot leaning. Independent torso-IMU validation now narrows this further; see below. A single static pose cannot
  determine yaw or a complete mounting transform, so the residual was not fitted away. The nominal axis correction and its remaining
  error are documented below; real Navigation direction still needs validation.
- The real camera and LowState use approximate PC receipt timestamps. The robot's
  onboard clock was not synchronized; it was not changed. Hardware capture
  synchronization, delivery latency and camera extrinsics need further validation.
- Intrinsics came from the connected device, but optical calibration accuracy still
  needs a multi-pose board check. Marker recognition alone does not certify robot
  relative coordinates or safe motion.
- No real walking, manipulation or policy transition was tested. The isolated sim
  check held the robot suspended and tested sensors/GLIM only. Simulator truth was
  not used for recognition or target construction.

## Camera coordinates and timing: 2026-10-23 follow-up

The independent torso IMU is `rt/secondary_imu`, as named in Unitree's
`example/g1/low_level/g1_ankle_swing_example.cpp`. Its gravity direction agreed
with pelvis IMU plus measured waist FK within **0.45–0.57°** in two static checks.
SDK mode was PR (0); waist indices 12/13/14 are yaw/roll/pitch. This supports the
current joint mapping and root-to-torso chain, without certifying full camera or
LiDAR extrinsics. Before the nominal-axis correction below, the LiDAR mount disagreed by
**13.27°** in the later pose. A trial 180° rotation about the other axis reduced the discrepancy to
about 6.1° but did not eliminate it. The subsequent nominal correction is
supported by a published G1 sensor model, as detailed below; no residual was fitted away.
The LiDAR floor normal and its own IMU agreed within **0.33°**. The operator confirmed factory mounting without modifications. Independent
geometry is still needed to certify the full mounting transform. The configured internal LiDAR/IMU translation is also
not independently calibrated by these static gravity checks.

The native camera was calibrated from its device profile. Marker ID 0 (12 cm)
had a reprojection RMSE of **0.17 px** at 960×540. The latest marker-center estimate
was camera optical XYZ **(-0.158, -0.212, 0.951) m**, and horizontal pelvis-relative
XYZ **(0.881, -0.116, -0.052) m**. Optical X points right; root_yaw Y points left.
Camera-left does not necessarily mean pelvis-left: the waist yaw was approximately
-19.3°, and the camera was pitched downward. The transform explicitly includes
measured waist joints and pelvis IMU tilt, without GLIM or simulator world poses.

These values describe the **marker center**, not the `carry_box` object origin.
The supplied object-to-marker translation is -0.1305 m along object X. That
simulation object geometry must be measured/confirmed for the actual box, as must
table marker offsets. A small reprojection error only checks PnP consistency;
it does not establish the accuracy of camera mounting, marker size, object offsets
or root-relative position. Independent measurements of marker center relative to
the pelvis remain required for an absolute accuracy claim. Integer detection
corners were unchanged in this stationary sample; their zero variation must not
be interpreted as zero measurement uncertainty. 154 distinct image payloads were
received among 183 audited frames, excluding a completely frozen stream.

### Receipt and processing latency

A Python ROS Image assignment was checking every byte of the 960×540 image.
Using the message's typed unsigned-byte array avoids that per-pixel Python scan,
without changing image bytes. Camera stamps now denote software receipt before
JPEG decoding. In two 15-second sensor-only runs:

| Stream | Earlier median | Updated median | Updated p95 |
| --- | ---: | ---: | ---: |
| Camera image receipt minus header | 98.4 ms | 11.0 ms | 16.4 ms |
| Marker-result receipt minus image header | 136.3 ms | 27.6 ms | 36.7 ms |
| Livox IMU receipt minus header | — | 0.4 ms | 15.0 ms |
| Livox scan receipt minus header | — | 101.8 ms | 115.9 ms |
| GLIM odometry receipt minus header | — | 110.8 ms | 127.7 ms |

LiDAR headers refer to scan start; roughly 100 ms includes scan accumulation at
10 Hz. These are observed software receipt delays under this test load, not
hardware exposure latency or a timing guarantee. The camera stamp moved earlier,
so its improved figure also includes decoding rather than concealing it.
All **183/183 images and 150/150 GLIM samples** matched measured-state history;
no observed stream rewound. Successful interpolation is not proof of physical
capture synchronization. A separate LowState capture measured approximately
1000 Hz, p95 receipt interval 1.71 ms, and receipt-minus-tick residual p05/p95
-0.58/+0.62 ms after removing the median clock offset. Absolute DDS transport
latency cannot be inferred from that offset.

Viser retains a short marker-observation history so delayed GLIM odometry can
catch up with an image. It chooses the newest fresh timestamp-matched observation
rather than extrapolating the newest image or refreshing an old marker's age.

Repeat with the existing sensor processes running:

```bash
python scripts/inspect_sensors.py --environment real --network YOUR_ROBOT_INTERFACE --duration 15 --output sensor_report.json
python scripts/inspect_imu_mount.py --network YOUR_ROBOT_INTERFACE --duration 10 --output mount_report.json
```

The first tool checks ID 0 and a 0.12 m marker by default; use `--marker-id` and
`--marker-size` for another target. The second subscribes only to the two DDS IMUs
and Livox IMU, writes its observations, and never changes a calibration or commands
motors. Actual GLIM **map** publication/display validation is out of scope at the
user's request; the scan-cloud display was tested.

### Nominal MID360 axis correction

The previous mount applied Ry(pi), reversing raw X and Z. The nominal stock
mount now applies **Ry(0.0401425728) × Rx(pi)**, retaining the official URDF pitch
and position but reversing raw Y and Z. The raw sensor X axis is approximately
forward, Y right, Z downward. This is consistent with the independently published
[NVIDIA G1 MID360 site](https://github.com/NVlabs/GR00T-WholeBodyControl/blob/main/motionbricks/assets/skeletons/g1/g1.xml),
whose quaternion is approximately `(0, -0.999794, 0, 0.020295)` in wxyz order.
Its orientation supports the axis correction; its slightly different translation
was not copied. The factory-mount confirmation and measured gravity signs support
this nominal convention, but they are not a six-axis extrinsic calibration.

Both MJCF LiDAR/IMU sites and the generated shared FK YAML use the correction.
C++ Navigation, Python visualization and simulated raw data share the resulting
LiDAR model; the external Livox driver and GLIM source were not patched.
The camera mount is unchanged. Tests check forward/right/down raw axes, simulated
specific force, and agreement between generated Python/C++ transforms.
On the **same recorded real samples**, gravity disagreement falls from **13.27°
to 6.04°**. This remaining discrepancy is **unresolved**, not a fitted correction
or an acceptable calibration threshold. A multi-orientation observation or an
independent factory sensor-axis/extrinsic measurement is needed to determine it.
No real walking has been tested with this correction.

The operator clarified that the approximate **0.80 m forward, 0.20 m left**
estimate uses the **head camera**, not the pelvis, as reference. Using the head
camera origin and the horizontal projection of its optical forward axis gives
**0.838 m forward, 0.123 m left** for the audited sample: differences of +3.8 cm
and -7.7 cm from the rough estimate. Optical XYZ remains (-0.158, -0.212, 0.951) m;
optical depth is not horizontal forward distance because the camera points down.
The model attaches the fixed head-camera mount to the torso kinematic chain; it
does not imply a camera physically mounted on the chest. The supplied estimate
is too approximate, and lacks height, to certify or fit absolute camera mounting
or pelvis-relative position. No camera extrinsic was tuned to force agreement.

### Manual head-hinge follow-up

The operator confirmed that both camera and Livox move with an unmeasured head
hinge. A manual adjustment changed model/Livox gravity disagreement from 5.96
to 14.69 degrees while pelvis-IMU foot tilt stayed nearly unchanged. A second
adjustment reduced disagreement to 0.687 degrees; a horizontal marker then had
1.094-degree model-derived tilt. This supersedes the earlier six-degree residual
for the current physical head setting. Keep this setting fixed/reproducible; the
model does not track manual hinge movement. This is static consistency evidence,
not a full six-axis extrinsic calibration. No actuation was commanded.
