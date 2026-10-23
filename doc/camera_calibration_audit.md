# Real camera: calibrated 960×540 output

The Unitree JPEG bridge publishes **960×540** Image and CameraInfo, matching
the simulator's output contract. It keeps the robot's existing camera service.
The network JPEG remains at its original resolution: local resizing does not
reduce robot-to-PC traffic.

## Start

In the camera terminal:

```bash
source /opt/ros/humble/setup.bash
cd /path/to/unitree_rl_mjlab
source /path/to/TaskPromptRL/venv-tprl/bin/activate
python scripts/camera_opencv.py YOUR_ROBOT_INTERFACE --no-display \
  --calibration aruco/calibrations/d435i_243122071510_color_1920x1080.yaml
```

The file above is included in this repository and is for the checked D435i
serial **243122071510** only. For another camera, replace it with that device's
calibration for the **received JPEG before resizing**.
The decoded dimensions must exactly match that calibration; a mismatch stops
publication. No default or guessed intrinsics are used.
The bridge resizes the complete 16:9 image to 960×540 without cropping,
stretching or padding. A 1920×1080 input is reduced by 1/2 in each dimension.
The full field of view is preserved: approximately 70.5° horizontal and 43.4°
vertical for the checked camera. Non-16:9 inputs are rejected instead of silently
changing aspect ratio.

K and P receive the same pixel transformation as the image, including OpenCV's
pixel-center convention. D and R remain unchanged; no undistortion is performed.
Recognition receives the transformed CameraInfo. Image and CameraInfo use the
same frame and receipt timestamp. Static drawing TF remains off by default.
The sim camera supplies its own calibrated model parameters: the default model now uses approximately the same color field of view.
Its ideal centered principal point still differs from the real calibrated one.

## Calibration obtained from the connected robot

The checked camera is **D435i serial 243122071510**, firmware 5.15.1.55.
Its native color profile is 1920×1080. Calibration read from the device using
`rs-enumerate-devices -c` is stored in
[the serial-specific YAML](../aruco/calibrations/d435i_243122071510_color_1920x1080.yaml).
The startup command above uses this file. Use it only for this camera and this
delivered profile.

The factory reports Inverse Brown Conrady with all five coefficients zero;
zero-distortion plumb_bob is equivalent here. Do not apply that equivalence to
nonzero inverse-model coefficients on another camera.
The inspected factory pipeline captures YUY2 at 1920×1080, converts to NV12
and JPEG-encodes without an explicit geometric resize on the JPEG branch.
Actual received JPEG dimensions were verified. This supports using the native
factory intrinsics, but is not a full optical calibration of delivered images.

A 12 cm marker in the full-frame 960×540 output gave optical depth 1.012 m
and Euclidean distance 1.085 m, compared with the operator's approximate 1.05 m.
After repositioning, marker ID 0 remained detectable at optical depth 0.945 m.
These are coarse sanity checks, not calibration-accuracy claims. Reserve known-size
board images at multiple distances, tilts and image locations for metric and
reprojection validation.

## Evidence and timing

`--save-on-exit /path/to/frame.png` saves the final **output** image with a JSON
sidecar containing native calibration/hash, source/output dimensions, transformed
K and processing description. The bridge uses software receipt time because
GetImageSample exposes JPEG bytes, not a verified hardware capture timestamp.
The real measured-state reader also uses PC receipt time; this is an approximation
whose latency still needs characterization.

Factory intrinsics require access to the USB-connected computer; routine JPEG
capture runs on the operator PC without SSH. The SDK does not expose a camera
resolution setter. Native 640×480 is supported by the device, but replacing
the factory capture service is outside this workflow.
