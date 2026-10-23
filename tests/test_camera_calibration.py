"""Synthetic calibration fixtures only; tests never connect to a camera/robot."""
import contextlib
import io
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import camera_opencv as camera


def synthetic_calibration():
    # Deliberately invented test geometry, not a deploy calibration.
    return {
        'image_width': 640,
        'image_height': 480,
        'camera_matrix': {'data': [500., 0., 320., 0., 510., 240., 0., 0., 1.]},
        'rectification_matrix': {'data': np.eye(3).reshape(-1).tolist()},
        'projection_matrix': {'data': [500., 0., 320., 0., 0., 510., 240., 0., 0., 0., 1., 0.]},
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {'data': [.01, -.02, .001, -.001, .003]},
    }


class CalibrationGeometryTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / 'synthetic.yaml'
        self.document = synthetic_calibration()
        self.path.write_text(yaml.safe_dump(self.document))
        self.calibration = camera.load_calibration(self.path)

    def test_missing_calibration_fails_before_sdk_initialization(self):
        with patch.object(camera, 'create_video_client') as create, contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as error:
                camera.main(['lo', '--no-display'])
            self.assertEqual(error.exception.code, 2)
            create.assert_not_called()

    def test_old_resolution_rejected_before_sdk_initialization(self):
        self.document.update(image_width=1920, image_height=1080)
        self.path.write_text(yaml.safe_dump(self.document))
        with patch.object(camera, 'create_video_client') as create, contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(camera.main(['--calibration', str(self.path)]), 2)
            create.assert_not_called()

    def test_source_resolution_must_be_640_by_480(self):
        for width, height in ((1920, 1080), (1280, 960), (640, 360), (0, 480)):
            with self.subTest(size=(width, height)), self.assertRaisesRegex(ValueError, 'configure the source stream'):
                camera.camera_info_message(self.calibration, width, height, 'camera', None)

    def test_wrong_sdk_frame_is_never_published(self):
        node = Mock(calibration=self.calibration, last_image=None)
        node.client.GetImageSample.return_value = (0, b'JPEG')
        with (
            patch.object(camera.rclpy, 'ok', return_value=True),
            patch.object(camera.cv2, 'imdecode', return_value=np.zeros((720, 1280, 3), dtype=np.uint8)),
            self.assertRaisesRegex(ValueError, 'configure the source stream'),
        ):
            camera.G1CameraPublisher.run(node)
        node.info_publisher.publish.assert_not_called()
        node.image_publisher.publish.assert_not_called()

    def test_matching_profile_preserves_actual_parameters(self):
        from builtin_interfaces.msg import Time
        message = camera.camera_info_message(self.calibration, 640, 480, 'camera', Time(sec=12))
        image = camera.image_message(np.zeros((480, 640, 3), dtype=np.uint8), 'camera', Time(sec=12))
        self.assertEqual((message.width, message.height), (image.width, image.height))
        self.assertEqual(message.header, image.header)
        np.testing.assert_array_equal(message.k, self.document['camera_matrix']['data'])
        np.testing.assert_array_equal(message.d, self.document['distortion_coefficients']['data'])

    def test_no_resize_escape_hatch_or_default_tf(self):
        args = camera.parse_args(['--calibration', str(self.path)])
        self.assertFalse(args.publish_static_tf)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
            camera.parse_args(['--calibration', str(self.path), '--allow-intrinsics-resize'])
        self.assertEqual(error.exception.code, 2)

    def test_bad_intrinsics_rejected(self):
        self.document['camera_matrix']['data'][0] = float('nan')
        self.path.write_text(yaml.safe_dump(self.document))
        with self.assertRaisesRegex(ValueError, 'finite'):
            camera.load_calibration(self.path)
