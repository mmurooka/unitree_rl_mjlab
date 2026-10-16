import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import yaml

from frame_pose_publisher import (
    CompatibleArucoDetector,
    estimate_marker_pose,
    image_arrays,
    load_config,
    matrix_pose,
    pose_matrix,
)


class PoseMathTest(unittest.TestCase):
    def test_rgb_image_is_converted_for_detection_and_display(self):
        message = SimpleNamespace(
            encoding="rgb8",
            width=2,
            height=1,
            step=8,
            data=bytes([255, 0, 0, 0, 255, 0, 99, 99]),
        )

        gray, display = image_arrays(message)

        self.assertEqual(gray.shape, (1, 2))
        np.testing.assert_array_equal(
            display,
            np.array([[[0, 0, 255], [0, 255, 0]]], dtype=np.uint8),
        )

    def test_pose_matrix_round_trip(self):
        expected = np.array([1.2, -0.3, 0.8, 0.1, -0.2, 0.3, 0.9])
        actual = np.asarray(matrix_pose(pose_matrix(expected)))
        expected[3:] /= np.linalg.norm(expected[3:])
        np.testing.assert_allclose(actual, expected, atol=1.0e-12)

    def test_object_to_marker_is_inverted(self):
        config = {
            "frame_pose_list": {
                "enabled": True,
                "source": "aruco",
                "topic": "/poses",
                "aruco": {
                    "image_topic": "/image",
                    "camera_info_topic": "/info",
                    "reference_frame": "base",
                    "markers": {
                        7: {
                            "frame_name": "box",
                            "marker_size": 0.1,
                            "object_to_marker": {
                                "translation": [0.2, 0.0, 0.0],
                                "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
                            },
                        }
                    },
                },
            }
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.yaml"
            path.write_text(yaml.safe_dump(config), encoding="utf-8")
            marker = load_config(path).markers[7]
        np.testing.assert_allclose(marker.marker_to_object[:3, 3], [-0.2, 0.0, 0.0])

    def test_marker_pose_uses_legacy_coordinate_convention(self):
        camera_matrix = np.array(
            [[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]]
        )
        distortion = np.zeros(5)
        marker_size = 0.12
        object_points = np.array(
            [
                [-0.06, 0.06, 0.0],
                [0.06, 0.06, 0.0],
                [0.06, -0.06, 0.0],
                [-0.06, -0.06, 0.0],
            ]
        )
        expected_rotation = np.array([np.pi, 0.0, 0.0])
        expected_translation = np.array([0.1, -0.05, 1.2])
        image_points = cv2.projectPoints(
            object_points,
            expected_rotation,
            expected_translation,
            camera_matrix,
            distortion,
        )[0]

        actual = estimate_marker_pose(
            image_points, marker_size, camera_matrix, distortion
        )

        self.assertIsNotNone(actual)
        actual_rotation, actual_translation = actual
        np.testing.assert_allclose(actual_translation, expected_translation, atol=1.0e-3)
        np.testing.assert_allclose(
            cv2.Rodrigues(actual_rotation)[0],
            cv2.Rodrigues(expected_rotation)[0],
            atol=1.0e-3,
        )

    def test_detector_accepts_generated_marker(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        if hasattr(cv2.aruco, "generateImageMarker"):
            marker = cv2.aruco.generateImageMarker(dictionary, 3, 200)
        else:
            marker = cv2.aruco.drawMarker(dictionary, 3, 200)
        image = np.full((300, 300), 255, dtype=np.uint8)
        image[50:250, 50:250] = marker

        _, ids, _ = CompatibleArucoDetector(dictionary).detect(image)

        self.assertIsNotNone(ids)
        self.assertEqual(ids.reshape(-1).tolist(), [3])


if __name__ == "__main__":
    unittest.main()
