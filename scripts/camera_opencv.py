#!/usr/bin/env python3
"""Publish the G1 front-camera stream as ROS 2 Image and CameraInfo topics."""

from __future__ import annotations

import argparse
from array import array
from dataclasses import dataclass, replace
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

import cv2
import numpy as np
import yaml

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster


CAMERA_WIDTH = 960
CAMERA_HEIGHT = 540
DEFAULT_IMAGE_TOPIC = "/camera/camera/color/image_raw"
DEFAULT_CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info"
DEFAULT_FRAME_ID = "g1_front_camera_optical_frame"


@dataclass(frozen=True)
class StaticCameraTransform:
    parent_frame: str
    translation: tuple[float, float, float]
    rotation_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class CameraCalibration:
    width: int
    height: int
    distortion_model: str
    distortion: tuple[float, ...]
    camera_matrix: tuple[float, ...]
    rectification_matrix: tuple[float, ...]
    projection_matrix: tuple[float, ...]
    static_transform: StaticCameraTransform | None = None


def _finite_values(value: Any, length: int | None, name: str) -> tuple[float, ...]:
    try:
        values = tuple(float(item) for item in value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be a list of numbers") from error
    if length is not None and len(values) != length:
        raise ValueError(f"{name} must contain {length} values")
    if not all(math.isfinite(item) for item in values):
        raise ValueError(f"{name} must contain only finite values")
    return values


def _matrix_data(
    document: dict[str, Any], name: str, length: int | None
) -> tuple[float, ...]:
    matrix = document.get(name)
    if not isinstance(matrix, dict):
        raise ValueError(f"{name} must be a mapping")
    return _finite_values(matrix.get("data"), length, f"{name}.data")


def load_calibration(path: Path) -> CameraCalibration:
    """Load a ROS camera_calibration-compatible YAML file."""
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError("calibration document must be a mapping")

    width = int(document.get("image_width", 0))
    height = int(document.get("image_height", 0))
    if width <= 0 or height <= 0:
        raise ValueError("calibration dimensions must be positive")

    distortion = _matrix_data(document, "distortion_coefficients", length=None)
    if not distortion:
        raise ValueError("distortion_coefficients.data must not be empty")
    camera_matrix = _matrix_data(document, "camera_matrix", 9)
    rectification_matrix = _matrix_data(document, "rectification_matrix", 9)
    projection_matrix = _matrix_data(document, "projection_matrix", 12)
    if camera_matrix[0] <= 0.0 or camera_matrix[4] <= 0.0:
        raise ValueError("camera_matrix focal lengths must be positive")

    distortion_model = str(document.get("distortion_model", "plumb_bob")).strip()
    if not distortion_model:
        raise ValueError("distortion_model must not be empty")

    static_transform = None
    transform_document = document.get("static_transform")
    if transform_document is not None:
        if not isinstance(transform_document, dict):
            raise ValueError("static_transform must be a mapping")
        parent_frame = str(transform_document.get("parent_frame", "")).strip()
        if not parent_frame:
            raise ValueError("static_transform.parent_frame must not be empty")
        translation = _finite_values(
            transform_document.get("translation"),
            3,
            "static_transform.translation",
        )
        rotation = np.asarray(
            _finite_values(
                transform_document.get("rotation_xyzw"),
                4,
                "static_transform.rotation_xyzw",
            ),
            dtype=np.float64,
        )
        rotation_norm = float(np.linalg.norm(rotation))
        if rotation_norm < 1.0e-12:
            raise ValueError("static_transform.rotation_xyzw must not be zero")
        rotation /= rotation_norm
        static_transform = StaticCameraTransform(
            parent_frame=parent_frame,
            translation=translation,
            rotation_xyzw=tuple(float(value) for value in rotation),
        )
    return CameraCalibration(
        width=width,
        height=height,
        distortion_model=distortion_model,
        distortion=distortion,
        camera_matrix=camera_matrix,
        rectification_matrix=rectification_matrix,
        projection_matrix=projection_matrix,
        static_transform=static_transform,
    )


def matching_intrinsics(
    calibration: CameraCalibration, width: int, height: int
) -> tuple[list[float], list[float]]:
    """Require calibration for the exact image profile; never guess source intrinsics."""
    if (width, height) != (calibration.width, calibration.height):
        raise ValueError(
            f"image {width}x{height} does not match calibration "
            f"{calibration.width}x{calibration.height}"
        )
    return list(calibration.camera_matrix), list(calibration.projection_matrix)



def prepare_image(
    image: np.ndarray, calibration: CameraCalibration,
) -> tuple[np.ndarray, CameraCalibration]:
    """Resize the complete 16:9 image, preserving field of view and pixel centers."""
    height, width = image.shape[:2]
    matching_intrinsics(calibration, width, height)
    if width * CAMERA_HEIGHT != height * CAMERA_WIDTH:
        raise ValueError("source must be 16:9 for aspect-preserving 960x540 output")
    sx, sy = CAMERA_WIDTH / width, CAMERA_HEIGHT / height
    affine = np.array([[sx, 0., (sx - 1.) / 2.],
                       [0., sy, (sy - 1.) / 2.],
                       [0., 0., 1.]])
    output = cv2.resize(image, (CAMERA_WIDTH, CAMERA_HEIGHT), interpolation=cv2.INTER_AREA)
    transformed = replace(
        calibration, width=CAMERA_WIDTH, height=CAMERA_HEIGHT,
        camera_matrix=tuple((affine @ np.asarray(calibration.camera_matrix).reshape(3, 3)).flat),
        projection_matrix=tuple((affine @ np.asarray(calibration.projection_matrix).reshape(3, 4)).flat),
    )
    return output, transformed

def camera_info_message(
    calibration: CameraCalibration,
    width: int,
    height: int,
    frame_id: str,
    stamp: Any,
) -> CameraInfo:
    camera_matrix, projection_matrix = matching_intrinsics(calibration, width, height)
    message = CameraInfo()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.width = width
    message.height = height
    message.distortion_model = calibration.distortion_model
    message.d = list(calibration.distortion)
    message.k = camera_matrix
    message.r = list(calibration.rectification_matrix)
    message.p = projection_matrix
    return message


def image_message(image: np.ndarray, frame_id: str, stamp: Any) -> Image:
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise ValueError("camera image must be an 8-bit BGR image")
    height, width = image.shape[:2]
    contiguous = np.ascontiguousarray(image)
    message = Image()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.height = height
    message.width = width
    message.encoding = "bgr8"
    message.is_bigendian = False
    message.step = width * 3
    message.data = array("B", contiguous.tobytes())
    return message


def static_transform_message(
    transform: StaticCameraTransform, child_frame: str, stamp: Any
) -> TransformStamped:
    if transform.parent_frame == child_frame:
        raise ValueError("static camera TF parent and child frames must differ")
    message = TransformStamped()
    message.header.stamp = stamp
    message.header.frame_id = transform.parent_frame
    message.child_frame_id = child_frame
    message.transform.translation.x = transform.translation[0]
    message.transform.translation.y = transform.translation[1]
    message.transform.translation.z = transform.translation[2]
    message.transform.rotation.x = transform.rotation_xyzw[0]
    message.transform.rotation.y = transform.rotation_xyzw[1]
    message.transform.rotation.z = transform.rotation_xyzw[2]
    message.transform.rotation.w = transform.rotation_xyzw[3]
    return message


class G1CameraPublisher(Node):
    def __init__(
        self,
        client: Any,
        calibration: CameraCalibration,
        image_topic: str,
        camera_info_topic: str,
        frame_id: str,
        publish_static_tf: bool,
        display: bool,
        window_name: str,
    ) -> None:
        super().__init__("g1_camera_publisher")
        self.client = client
        self.calibration = calibration
        self.frame_id = frame_id
        self.display_enabled = display
        self.window_name = window_name
        self.window_initialized = False
        self.last_warning = -math.inf
        self.last_image: np.ndarray | None = None
        self.image_publisher = self.create_publisher(
            Image, image_topic, qos_profile_sensor_data
        )
        self.info_publisher = self.create_publisher(
            CameraInfo, camera_info_topic, qos_profile_sensor_data
        )
        self.tf_broadcaster: StaticTransformBroadcaster | None = None
        if publish_static_tf and calibration.static_transform is not None:
            self.tf_broadcaster = StaticTransformBroadcaster(self)
            self.tf_broadcaster.sendTransform(
                static_transform_message(
                    calibration.static_transform,
                    frame_id,
                    self.get_clock().now().to_msg(),
                )
            )
        self.get_logger().info(
            f"publishing G1 camera on {image_topic} and {camera_info_topic}; "
            f"frame_id={frame_id}, display={'on' if display else 'off'}"
        )
        if self.tf_broadcaster is not None:
            self.get_logger().info(
                "publishing static camera TF "
                f"{calibration.static_transform.parent_frame} -> {frame_id}"
            )

    def warn_throttled(self, message: str, seconds: float = 2.0) -> None:
        now = time.monotonic()
        if now - self.last_warning >= seconds:
            self.get_logger().warning(message)
            self.last_warning = now

    def show(self, image: np.ndarray) -> bool:
        """Display one frame. Return false when the user asks to stop."""
        if not self.display_enabled:
            return True
        try:
            if not self.window_initialized:
                cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
                self.window_initialized = True
            cv2.imshow(self.window_name, image)
            key = cv2.waitKey(1) & 0xFF
            return key not in (27, ord("q"))
        except cv2.error as error:
            self.display_enabled = False
            self.window_initialized = False
            self.warn_throttled(f"camera display disabled: {error}")
            return True

    def run(self) -> None:
        while rclpy.ok():
            code, data = self.client.GetImageSample()
            if not rclpy.ok():
                break
            if code != 0:
                self.warn_throttled(f"GetImageSample failed with code {code}")
                time.sleep(0.05)
                rclpy.spin_once(self, timeout_sec=0.0)
                continue
            if data is None or len(data) == 0:
                self.warn_throttled("GetImageSample returned an empty image")
                continue

            # Software receipt time, before decode/resize/serialization.
            stamp = self.get_clock().now().to_msg()
            encoded = np.frombuffer(bytes(data), dtype=np.uint8)
            image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if image is None:
                self.warn_throttled("failed to decode the camera JPEG frame")
                continue

            height, width = image.shape[:2]
            if self.last_image is None:
                self.get_logger().info(
                    f"SDK JPEG decoded as {width}x{height}; calibration "
                    f"{self.calibration.width}x{self.calibration.height}; "
                    "bridge resizes the full 16:9 image to 960x540 with transformed K/P"
                )
            image, output_calibration = prepare_image(image, self.calibration)
            self.info_publisher.publish(
                camera_info_message(
                    output_calibration, CAMERA_WIDTH, CAMERA_HEIGHT, self.frame_id, stamp,
                )
            )
            self.image_publisher.publish(image_message(image, self.frame_id, stamp))
            self.output_calibration = output_calibration
            self.last_image = image
            rclpy.spin_once(self, timeout_sec=0.0)
            if not self.show(image):
                break

    def destroy_node(self) -> None:
        if self.window_initialized:
            cv2.destroyWindow(self.window_name)
            self.window_initialized = False
        super().destroy_node()


def create_video_client(interface: str | None, timeout: float) -> Any:
    # Import lazily so calibration/tests do not require the Unitree SDK runtime.
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.video.video_client import VideoClient

    if interface:
        ChannelFactoryInitialize(0, interface)
    else:
        ChannelFactoryInitialize(0)
    client = VideoClient()
    client.SetTimeout(timeout)
    client.Init()
    return client


def parse_args(arguments: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Bridge the Unitree G1 front camera to ROS 2 topics."
    )
    parser.add_argument(
        "interface",
        nargs="?",
        help="network interface connected to the robot (for example enp2s0)",
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        required=True,
        help="calibration for the actual received JPEG dimensions, before local resize; no default intrinsics",
    )
    parser.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC)
    parser.add_argument("--camera-info-topic", default=DEFAULT_CAMERA_INFO_TOPIC)
    parser.add_argument("--frame-id", default=DEFAULT_FRAME_ID)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--window-name", default="G1 front camera")
    parser.add_argument("--publish-static-tf", action="store_true",
                        help="optional legacy drawing TF; recognition uses measured model FK")
    parser.add_argument(
        "--no-publish-static-tf",
        action="store_false",
        dest="publish_static_tf",
        help="do not publish the camera extrinsic from the calibration YAML",
    )
    parser.add_argument(
        "--no-display",
        action="store_false",
        dest="display",
        help="publish without opening an OpenCV window",
    )
    parser.add_argument(
        "--save-on-exit",
        type=Path,
        help="optionally save the last decoded frame when the node exits",
    )
    parser.set_defaults(display=True, publish_static_tf=False)
    args = parser.parse_args(arguments)
    if not args.image_topic.strip() or not args.camera_info_topic.strip():
        parser.error("ROS topic names must not be empty")
    if not args.frame_id.strip():
        parser.error("--frame-id must not be empty")
    if not math.isfinite(args.timeout) or args.timeout <= 0.0:
        parser.error("--timeout must be a finite positive number")
    return args


def main(arguments: list[str] | None = None) -> int:
    args = parse_args(arguments)
    try:
        calibration = load_calibration(args.calibration)
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f"calibration error: {error}", file=sys.stderr)
        return 2
    try:
        client = create_video_client(args.interface, args.timeout)
    except Exception as error:
        print(f"Unitree camera initialization failed: {error}", file=sys.stderr)
        return 2

    rclpy.init(args=None)
    node = G1CameraPublisher(
        client=client,
        calibration=calibration,
        image_topic=args.image_topic,
        camera_info_topic=args.camera_info_topic,
        frame_id=args.frame_id,
        publish_static_tf=args.publish_static_tf,
        display=args.display,
        window_name=args.window_name,
    )
    result = 0
    try:
        node.run()
    except ValueError as error:
        print(f"camera profile error: {error}", file=sys.stderr)
        result = 2
    except KeyboardInterrupt:
        pass
    except (rclpy.executors.ExternalShutdownException, rclpy._rclpy_pybind11.RCLError):
        if rclpy.ok():
            raise
    finally:
        if args.save_on_exit is not None and node.last_image is not None:
            if not cv2.imwrite(str(args.save_on_exit), node.last_image):
                node.get_logger().error(f"failed to save {args.save_on_exit}")
            else:
                args.save_on_exit.with_suffix(args.save_on_exit.suffix + ".json").write_text(
                    json.dumps({
                        "calibration_path": str(args.calibration.resolve()),
                        "calibration_sha256": hashlib.sha256(args.calibration.read_bytes()).hexdigest(),
                        "calibration_yaml": args.calibration.read_text(),
                        "image_width": node.last_image.shape[1],
                        "image_height": node.last_image.shape[0],
                        "source_width": calibration.width,
                        "source_height": calibration.height,
                        "bridge_processing": "full-image aspect-preserving resize to 960x540 (OpenCV pixel centers)",
                        "output_camera_matrix": list(node.output_calibration.camera_matrix),
                        "stream": "Unitree VideoClient.GetImageSample JPEG",
                        "bridge_undistortion": False,
                        "sdk_rectification": "unknown; verify on hardware",
                    }, indent=2) + "\n"
                )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == "__main__":
    raise SystemExit(main())
