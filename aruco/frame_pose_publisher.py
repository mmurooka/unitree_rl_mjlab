#!/usr/bin/env python3
"""Publish ArUco-derived object poses using the /frame_pose_list contract."""

from __future__ import annotations

import argparse
from array import array
import json
import math
from dataclasses import dataclass
from pathlib import Path
import sys
import time
from typing import Any

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from robot_frames import SensorKinematics, DEFAULT_MODEL
from measured_robot_state import MeasuredRobotState


DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "simulate" / "config.yaml"


def _as_vector(value: Any, length: int, name: str) -> np.ndarray:
    vector = np.asarray(value, dtype=np.float64)
    if vector.shape != (length,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must contain {length} finite numbers")
    return vector


def quaternion_matrix(rotation_xyzw: Any) -> np.ndarray:
    """Return a homogeneous rotation matrix for an ROS-order quaternion."""
    x, y, z, w = _as_vector(rotation_xyzw, 4, "rotation_xyzw")
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-12:
        raise ValueError("rotation quaternion must not be zero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 0],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 0],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), 0],
            [0, 0, 0, 1],
        ],
        dtype=np.float64,
    )


def pose_matrix(pose: Any) -> np.ndarray:
    """Build a homogeneous matrix from [x,y,z,qx,qy,qz,qw]."""
    pose_vector = _as_vector(pose, 7, "pose")
    matrix = quaternion_matrix(pose_vector[3:])
    matrix[:3, 3] = pose_vector[:3]
    return matrix


def configured_pose_matrix(config: dict[str, Any], name: str) -> np.ndarray:
    translation = _as_vector(config.get("translation"), 3, f"{name}.translation")
    rotation = _as_vector(config.get("rotation_xyzw"), 4, f"{name}.rotation_xyzw")
    matrix = quaternion_matrix(rotation)
    matrix[:3, 3] = translation
    return matrix


def matrix_quaternion(matrix: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to a normalized [x,y,z,w] quaternion."""
    rotation = np.asarray(matrix, dtype=np.float64)[:3, :3]
    quaternion = np.empty(4, dtype=np.float64)
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion[:] = (
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
            0.25 * scale,
        )
    else:
        axis = int(np.argmax(np.diag(rotation)))
        if axis == 0:
            scale = math.sqrt(
                max(0.0, 1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
            ) * 2.0
            quaternion[:] = (
                0.25 * scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
            )
        elif axis == 1:
            scale = math.sqrt(
                max(0.0, 1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
            ) * 2.0
            quaternion[:] = (
                (rotation[0, 1] + rotation[1, 0]) / scale,
                0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
            )
        else:
            scale = math.sqrt(
                max(0.0, 1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
            ) * 2.0
            quaternion[:] = (
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                0.25 * scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            )
    quaternion /= np.linalg.norm(quaternion)
    # q and -q are equivalent; a non-negative w gives stable JSON output.
    if quaternion[3] < 0.0:
        quaternion *= -1.0
    return quaternion


def matrix_pose(matrix: np.ndarray) -> list[float]:
    quaternion = matrix_quaternion(matrix)
    return [
        float(matrix[0, 3]),
        float(matrix[1, 3]),
        float(matrix[2, 3]),
        *(float(value) for value in quaternion),
    ]


class CompatibleArucoDetector:
    """Hide the module-to-class ArUco API change in OpenCV 5."""

    def __init__(self, dictionary: Any) -> None:
        self.dictionary = dictionary
        if hasattr(cv2.aruco, "ArucoDetector"):
            parameters = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)
            self.parameters = None
        else:
            self.detector = None
            if hasattr(cv2.aruco, "DetectorParameters_create"):
                self.parameters = cv2.aruco.DetectorParameters_create()
            else:
                self.parameters = cv2.aruco.DetectorParameters()
            if not hasattr(cv2.aruco, "detectMarkers"):
                raise RuntimeError("this OpenCV build has no ArUco marker detector")

    def detect(self, image: np.ndarray) -> tuple[Any, Any, Any]:
        if self.detector is not None:
            return self.detector.detectMarkers(image)
        return cv2.aruco.detectMarkers(
            image, self.dictionary, parameters=self.parameters
        )


def estimate_marker_pose(
    corner: Any,
    marker_size: float,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> tuple[np.ndarray, np.ndarray] | None:
    """Estimate one marker pose without removed OpenCV ArUco pose helpers."""
    half_size = marker_size * 0.5
    # ArUco corners arrive clockwise from top-left. This point order preserves
    # the legacy estimatePoseSingleMarkers marker frame: +x right, +y up and
    # +z out of the printed face.
    object_points = np.array(
        [
            [-half_size, half_size, 0.0],
            [half_size, half_size, 0.0],
            [half_size, -half_size, 0.0],
            [-half_size, -half_size, 0.0],
        ],
        dtype=np.float64,
    )
    image_points = np.asarray(corner, dtype=np.float64).reshape(4, 2)
    success, rotation_vector, translation_vector = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        distortion,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not success:
        return None
    return rotation_vector.reshape(3), translation_vector.reshape(3)


@dataclass(frozen=True)
class Marker:
    marker_id: int
    frame_name: str
    marker_size: float
    marker_to_object: np.ndarray


@dataclass(frozen=True)
class PublisherConfig:
    topic: str
    image_topic: str
    camera_info_topic: str
    camera_frame: str
    reference_frame: str
    dictionary_id: int
    dictionary_name: str
    display: bool
    display_window: str
    markers: dict[int, Marker]


def load_config(path: Path, require_aruco_source: bool = True) -> PublisherConfig:
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    frame_config = document.get("frame_pose_list", {})
    if not frame_config.get("enabled", False):
        raise ValueError("frame_pose_list.enabled must be true")
    source = frame_config.get("source", "ground_truth")
    if require_aruco_source and source != "aruco":
        raise ValueError(
            "frame_pose_list.source is not 'aruco'; change it in the YAML "
            "to switch from ground truth"
        )

    aruco_config = frame_config.get("aruco", {})
    dictionary_name = str(aruco_config.get("dictionary", "DICT_4X4_50"))
    dictionary_id = getattr(cv2.aruco, dictionary_name, None)
    if dictionary_id is None:
        raise ValueError(f"unknown OpenCV ArUco dictionary: {dictionary_name}")

    markers: dict[int, Marker] = {}
    frame_names: set[str] = set()
    for raw_id, marker_config in aruco_config.get("markers", {}).items():
        marker_id = int(raw_id)
        frame_name = str(marker_config.get("frame_name", "")).strip()
        marker_size = float(marker_config.get("marker_size", 0.0))
        if marker_id < 0 or not frame_name or not math.isfinite(marker_size) or marker_size <= 0.0:
            raise ValueError(f"invalid marker configuration for ID {raw_id}")
        if frame_name in frame_names:
            raise ValueError(f"duplicate frame_name in marker map: {frame_name}")
        object_to_marker = configured_pose_matrix(
            marker_config.get("object_to_marker", {}),
            f"markers.{marker_id}.object_to_marker",
        )
        markers[marker_id] = Marker(
            marker_id=marker_id,
            frame_name=frame_name,
            marker_size=marker_size,
            marker_to_object=np.linalg.inv(object_to_marker),
        )
        frame_names.add(frame_name)
    if not markers:
        raise ValueError("at least one ArUco marker mapping is required")

    reference_frame = "root_yaw"
    values = {
        "topic": str(frame_config.get("topic", "/frame_pose_list")).strip(),
        "image_topic": str(aruco_config.get("image_topic", "")).strip(),
        "camera_info_topic": str(aruco_config.get("camera_info_topic", "")).strip(),
        "reference_frame": reference_frame,
        "display_window": str(
            aruco_config.get("display_window", "Robot camera / ArUco")
        ).strip(),
    }
    for name, value in values.items():
        if not value:
            raise ValueError(f"{name} must not be empty")
    return PublisherConfig(
        **values,
        camera_frame=str(aruco_config.get("camera_frame", "")).strip(),
        dictionary_id=int(dictionary_id),
        dictionary_name=dictionary_name,
        display=bool(aruco_config.get("display", False)),
        markers=markers,
    )


def image_arrays(message: Image) -> tuple[np.ndarray, np.ndarray]:
    """Return grayscale detection and BGR display images."""
    encoding = message.encoding.lower()
    formats = {
        "mono8": (1, None, cv2.COLOR_GRAY2BGR),
        "rgb8": (3, cv2.COLOR_RGB2GRAY, cv2.COLOR_RGB2BGR),
        "bgr8": (3, cv2.COLOR_BGR2GRAY, None),
        "rgba8": (4, cv2.COLOR_RGBA2GRAY, cv2.COLOR_RGBA2BGR),
        "bgra8": (4, cv2.COLOR_BGRA2GRAY, cv2.COLOR_BGRA2BGR),
    }
    if encoding not in formats:
        raise ValueError(f"unsupported image encoding: {message.encoding}")
    channels, gray_conversion, bgr_conversion = formats[encoding]
    packed_width = int(message.width) * channels
    if int(message.step) < packed_width:
        raise ValueError("image step is smaller than its packed row width")
    data = np.frombuffer(message.data, dtype=np.uint8)
    expected = int(message.height) * int(message.step)
    if data.size < expected:
        raise ValueError("image data is shorter than height * step")
    rows = data[:expected].reshape(int(message.height), int(message.step))
    pixels = rows[:, :packed_width]
    if channels == 1:
        gray = pixels.reshape(int(message.height), int(message.width)).copy()
        return gray, cv2.cvtColor(gray, bgr_conversion)
    source = pixels.reshape(int(message.height), int(message.width), channels)
    gray = cv2.cvtColor(source, gray_conversion)
    if bgr_conversion is None:
        display = source.copy()
    else:
        display = cv2.cvtColor(source, bgr_conversion)
    return gray, display


class ArucoFramePosePublisher(Node):
    def __init__(self, config: PublisherConfig, environment: str,
                 network: str | None = None, domain_id: int = 0,
                 model: Path = DEFAULT_MODEL) -> None:
        super().__init__("aruco_frame_pose_list")
        self.config = config
        self.environment = environment
        self.kinematics = SensorKinematics(model)
        self.measured = MeasuredRobotState(
            environment, lambda: self.get_clock().now().nanoseconds * 1e-9, network, domain_id)
        self.info_shape = None
        self.info_frame = None
        self.dictionary = cv2.aruco.getPredefinedDictionary(config.dictionary_id)
        self.marker_detector = CompatibleArucoDetector(self.dictionary)
        self.camera_matrix: np.ndarray | None = None
        self.distortion: np.ndarray | None = None
        self.last_warning: dict[str, float] = {}
        self.display_enabled = config.display
        self.display_initialized = False

        self.publisher = self.create_publisher(String, config.topic, 1)
        self.annotated_publisher = self.create_publisher(
            CompressedImage, config.topic + "/image/compressed", qos_profile_sensor_data
        )
        self.info_subscription = self.create_subscription(
            CameraInfo,
            config.camera_info_topic,
            self.on_camera_info,
            qos_profile_sensor_data,
        )
        self.image_subscription = self.create_subscription(
            Image, config.image_topic, self.on_image, qos_profile_sensor_data
        )
        marker_summary = ", ".join(
            f"{marker_id}:{marker.frame_name}"
            for marker_id, marker in sorted(config.markers.items())
        )
        transform_source = "model FK + measured joints/body IMU (no TF/GLIM)"
        self.get_logger().info(
            f"publishing {config.topic} from {config.image_topic}; "
            f"dictionary={config.dictionary_name}, markers=[{marker_summary}], "
            f"reference={config.reference_frame} via {transform_source}, "
            f"display={'on' if config.display else 'off'}"
        )

    def show_camera(self, image: np.ndarray, header) -> None:
        # Publish the exact detection frame, including empty detections and
        # unavailable FK. Visualization must not depend on localization.
        if self.annotated_publisher.get_subscription_count():
            ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                output = CompressedImage()
                output.header = header
                output.format = "bgr8; jpeg compressed bgr8"
                output.data = array("B", encoded.tobytes())
                self.annotated_publisher.publish(output)
        if not self.display_enabled:
            return
        try:
            if not self.display_initialized:
                cv2.namedWindow(self.config.display_window, cv2.WINDOW_AUTOSIZE)
                self.display_initialized = True
            cv2.imshow(self.config.display_window, image)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                cv2.destroyWindow(self.config.display_window)
                self.display_initialized = False
                self.display_enabled = False
                self.get_logger().info("camera display closed")
        except cv2.error as error:
            self.display_initialized = False
            self.display_enabled = False
            self.warn_throttled("display", f"camera display disabled: {error}")

    def destroy_node(self) -> None:
        self.measured.close()
        if self.display_initialized:
            cv2.destroyWindow(self.config.display_window)
            self.display_initialized = False
        super().destroy_node()

    def warn_throttled(self, key: str, message: str, seconds: float = 2.0) -> None:
        now = time.monotonic()
        if now - self.last_warning.get(key, -math.inf) >= seconds:
            self.get_logger().warning(message)
            self.last_warning[key] = now

    def on_camera_info(self, message: CameraInfo) -> None:
        matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        if not np.all(np.isfinite(matrix)) or matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
            self.warn_throttled("camera_info", "ignoring invalid CameraInfo intrinsics")
            return
        distortion = np.asarray(message.d, dtype=np.float64)
        if message.distortion_model not in ("plumb_bob", "rational_polynomial") or not np.isfinite(distortion).all():
            self.warn_throttled("camera_info", "unsupported/invalid CameraInfo distortion")
            return
        self.camera_matrix = matrix
        self.distortion = distortion
        self.info_shape = (int(message.width), int(message.height))
        self.info_frame = message.header.frame_id

    def reference_to_camera(self, camera_frame: str, stamp: float) -> np.ndarray | None:
        # Both stream names refer to the model's color optical mount. A new
        # physical camera needs an explicit model/calibration, not a guessed TF.
        if camera_frame not in ("camera_color_optical_frame", "g1_front_camera_optical_frame"):
            self.warn_throttled("camera_frame", f"unsupported optical frame: {camera_frame}")
            return None
        try:
            state = self.measured.at(stamp)
            return self.kinematics.horizontal_to_camera(state.q, state.quaternion)
        except ValueError as error:
            self.warn_throttled("measured_state", str(error))
            return None

    def on_image(self, message: Image) -> None:
        try:
            gray, display = image_arrays(message)
        except ValueError as error:
            self.warn_throttled("image", str(error))
            return

        corners, ids, _ = self.marker_detector.detect(gray)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(display, corners, ids)

        if self.camera_matrix is None or self.distortion is None:
            self.warn_throttled("no_info", "waiting for CameraInfo")
            self.show_camera(display, message.header)
            return
        camera_frame = self.config.camera_frame or message.header.frame_id
        if not camera_frame:
            self.warn_throttled("camera_frame", "Image header has no camera frame")
            self.show_camera(display, message.header)
            return
        if self.info_shape != (int(message.width), int(message.height)) or self.info_frame != message.header.frame_id:
            self.warn_throttled("camera_info_match", "Image and CameraInfo resolution/frame disagree")
            self.show_camera(display, message.header)
            return
        stamp = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9
        reference_to_camera = self.reference_to_camera(camera_frame, stamp)
        if reference_to_camera is None:
            self.show_camera(display, message.header)
            return

        detected: list[dict[str, Any]] = []
        if ids is not None:
            for corner, marker_id_value in zip(corners, ids.reshape(-1)):
                marker = self.config.markers.get(int(marker_id_value))
                if marker is None:
                    continue
                marker_pose = estimate_marker_pose(
                    corner, marker.marker_size, self.camera_matrix, self.distortion
                )
                if marker_pose is None:
                    continue
                rotation_vector, translation_vector = marker_pose
                cv2.drawFrameAxes(
                    display,
                    self.camera_matrix,
                    self.distortion,
                    rotation_vector,
                    translation_vector,
                    marker.marker_size * 0.5,
                )
                label_position = tuple(
                    np.rint(np.asarray(corner).reshape(4, 2)[0]).astype(int)
                )
                cv2.putText(
                    display,
                    marker.frame_name,
                    label_position,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )
                camera_to_marker = np.eye(4, dtype=np.float64)
                camera_to_marker[:3, :3] = cv2.Rodrigues(rotation_vector)[0]
                camera_to_marker[:3, 3] = translation_vector
                reference_to_object = (
                    reference_to_camera @ camera_to_marker @ marker.marker_to_object
                )
                if not np.all(np.isfinite(reference_to_object)):
                    continue
                detected.append(
                    {
                        "frame_name": marker.frame_name,
                        "rel_pose": matrix_pose(reference_to_object),
                    }
                )

        detected.sort(key=lambda item: item["frame_name"])
        stamp = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1.0e-9
        output = String()
        output.data = json.dumps(
            {
                "stamp": stamp,
                "reference_frame": self.config.reference_frame,
                "source": "aruco",
                "clock_domain": "simulation" if self.environment == "sim" else "ros",
                "state_time_source": "sdk_tick" if self.environment == "sim" else "dds_receipt_ros_time",
                "frame_pose_list": detected,
            },
            separators=(",", ":"),
        )
        self.publisher.publish(output)
        self.show_camera(display, message.header)


def parse_args(arguments: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Detect configured ArUco markers and publish object poses."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help=f"configuration YAML (default: {DEFAULT_CONFIG})",
    )
    parser.add_argument("--environment", choices=("sim", "real"), default="real",
                        help="state timestamp domain; sim uses SDK tick, real uses ROS receipt time")
    parser.add_argument("--network", default=None, help="Unitree DDS interface (e.g. lo in sim)")
    parser.add_argument("--domain-id", type=int, default=0)
    parser.add_argument("--sensor-model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--glim", action="store_true",
                        help="deprecated compatibility flag; recognition no longer uses GLIM")
    return parser.parse_args(arguments)


def main(arguments: list[str] | None = None) -> int:
    args = parse_args(arguments)
    try:
        config = load_config(args.config)
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f"configuration error: {error}", file=sys.stderr)
        return 2

    if args.glim:
        print("--glim is no longer needed: recognition uses measured joints and body IMU.", file=sys.stderr)

    rclpy.init(args=None)
    node = ArucoFramePosePublisher(config, args.environment, args.network, args.domain_id, args.sensor_model)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
