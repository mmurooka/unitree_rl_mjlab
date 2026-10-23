# Real camera: 640×480 and calibrated intrinsics

The required real color stream is **640×480**, matching the simulator's default
resolution. The real bridge does not use repository-supplied intrinsics. Supply
calibration for the actual camera and delivered stream explicitly; the former
unverified YAML is not available as a fallback.

## Current Unitree JPEG bridge

In each camera terminal, set the actual robot-facing interface (inspect names
with `ip -brief link` and replace this placeholder):

```bash
export ROBOT_INTERFACE=YOUR_ROBOT_INTERFACE
```

```bash
python scripts/camera_opencv.py "$ROBOT_INTERFACE" --no-display \
  --calibration /path/to/actual_640x480_calibration.yaml
```

`--calibration` is required. Missing/invalid calibration fails before initializing
the Unitree SDK. Both calibration dimensions and received JPEG dimensions must
be 640×480. A different received size stops the bridge before publishing that
image or CameraInfo. The bridge does not resize, crop, undistort or scale K/P.
Recognition uses measured model kinematics; static drawing TF is off by default.

The installed `VideoClient.GetImageSample` API only returns JPEG bytes. It has no
resolution-setting or intrinsic-calibration query. Enforcing 640×480 here does
**not** reconfigure the robot's camera or its JPEG server. That upstream stream
must be configured separately. Until a matching actual stream and calibration
are available, the bridge intentionally cannot run recognition with guessed values.

## Obtaining actual calibration

For direct access to the D435, configure the color stream to 640×480 and retrieve
intrinsics from the active `video_stream_profile.get_intrinsics()`. The
[RealSense SDK projection documentation](https://dev.realsenseai.com/docs/projection-in-realsense-sdk-2-0/)
describes per-stream parameters. This requires access to the device, typically
on the computer physically connected to it. A RealSense ROS camera driver can
publish Image and CameraInfo together there; this is a different acquisition
path from the Unitree JPEG bridge and is not automatically launched by this repo.

For Unitree JPEG delivery, the YAML must describe that exact delivered image.
Do not copy raw D435 intrinsics if the JPEG server crops, resizes, rectifies or
uses another stream. Establish the delivery profile or calibrate the delivered
images with a known-size board. A syntactically valid YAML cannot establish
optical accuracy. Do not replace missing evidence with nominal focal lengths
or zero distortion coefficients.

The acquisition choice (direct device access versus retaining Unitree JPEG) and
actual real-stream configuration still require confirmation. No physical camera
was connected for these changes.

## Validate and preserve evidence

Capture a known-size board at varied distances, tilts and image locations. Reserve
images for validation; check reprojection residuals and marker metric distance
against independent measurements. Retain camera serial/profile, calibration
provenance and the delivered-image processing description with the YAML.

```bash
python scripts/camera_opencv.py "$ROBOT_INTERFACE" --no-display \
  --calibration /path/to/actual_640x480_calibration.yaml \
  --save-on-exit /path/to/frame.png
```

On exit, a received valid image is saved with `frame.png.json` containing calibration
content/hash, decoded size and processing assumptions. Image timestamps are software
receipt times; hardware capture/transport timing remains to be measured.

The simulator derives CameraInfo from its MuJoCo camera model and does not use
this calibration file. Successful simulated ArUco recognition does not validate
real optics. `tests/test_camera_calibration.py` uses explicitly synthetic fixtures
to check missing-calibration rejection, exact profile matching and unchanged K/D;
it neither measures nor supplies real calibration.
