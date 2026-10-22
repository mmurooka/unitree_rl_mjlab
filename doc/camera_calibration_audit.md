# Camera calibration audit (2026-10-23)

The real bridge receives JPEG bytes through Unitree's
`VideoClient.GetImageSample`, decodes BGR with OpenCV, and publishes the decoded
size. It does not crop, resize, or undistort the image. Whether the upstream SDK
stream is rectified/cropped, and its physical camera/stream profile, cannot be
established without robot stream metadata or calibration images.

`aruco/g1_front_camera.yaml` describes 1920x1080, fx=1154.7042,
fy=865.0281, cx=964.4358, cy=538.3260, plumb_bob with five zero distortion terms.
K and P agree and have valid numerical form. The fx/fy ratio is about 1.335;
this could reflect nonuniform resizing, but does not establish that such resizing
occurred. Commit e56e3bb changed the focal lengths from the earlier initial values
1350/1350. Neither value set has calibration-image evidence in these repositories.
The YAML itself labels these as initial intrinsics. No checkerboard/calibration
image collection or corresponding residual report was found in the two repositories.
Therefore the numerical values are preserved, and accuracy remains unverified.

The simulator generates CameraInfo independently from the MuJoCo camera model.
A successful simulated ArUco result does not validate the real camera YAML.

The bridge now rejects differing image/calibration dimensions by default.
`--allow-intrinsics-resize` explicitly opts into scaling K/P for a verified
full-image resize, including nonuniform scaling; it cannot represent a crop,
a changed field of view, or rectification. Silent scaling previously concealed
these distinctions. The first frame logs actual dimensions. Static TF is optional
and off by default; recognition already uses measured model FK.

To preserve evidence once the robot is available:

```bash
python scripts/camera_opencv.py <interface> --no-display \
  --calibration /path/to/calibration.yaml --save-on-exit /path/to/frame.png
```

On exit this saves the image plus `frame.png.json` with exact calibration content,
hash, decoded dimensions and processing assumptions. Use a matching stream profile
for calibration. Capture a known-size board at varied distances, tilts and image
locations; reserve images for validation. Check reprojection residuals throughout
the image and marker metric distance against independent measurements. A valid
K/P or low training-set residual alone is insufficient. Update calibration only
with that evidence and preserve old/new settings and image provenance.

Offline checks: `tests/test_camera_calibration.py` verifies unchanged matching
resolution, explicit-resize projection equivalence, and rejection of implicit
scaling. These checks validate software geometry, not real optical calibration.
