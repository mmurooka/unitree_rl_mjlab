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
python3 aruco/frame_pose_publisher.py
```

Set `source: "ground_truth"` to return to the MuJoCo body-pose publisher. Only
the selected source publishes `/frame_pose_list`. The simulator always
broadcasts the moving `imu_yaw -> camera_link` transform needed by the ArUco
node.

The simulated scene uses ID 0 on the robot-facing side of `carry_box`, ID 1 on
top of `pickup_table`, and ID 2 on top of `place_table`. Regenerate their PNGs
after changing the dictionary or IDs with:

```bash
python3 aruco/generate_markers.py
```

## Real robot

Use the same node and YAML mapping with `source: "aruco"`. Set `image_topic`
and `camera_info_topic` to the physical camera's ROS 2 topics. Set
`reference_frame` to the robot-relative frame expected by Navigation and make
sure TF can resolve that frame to the image's optical frame.

If the camera driver has no TF, set `use_tf: false` and enter the calibrated
camera pose as `static_reference_to_camera` in ROS pose order
`[x,y,z,qx,qy,qz,qw]`. A fixed transform is suitable only when the camera is
rigid relative to the chosen reference frame.

`object_to_marker` is the marker coordinate frame's pose in the object's frame.
It accounts for a marker being mounted on a side or top surface, so downstream
code still receives the object's center pose rather than the marker pose.
`marker_size` is the black marker's outer-edge length, excluding its white quiet
zone. For the supplied images, print the 12 cm marker on a 15 cm white square
and mount it with the orientation described by `object_to_marker`.
