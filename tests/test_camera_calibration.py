"""Checks require ROS Python; no camera or robot connection is opened."""
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from camera_opencv import DEFAULT_CALIBRATION, load_calibration, scaled_intrinsics, parse_args


class CalibrationGeometryTest(unittest.TestCase):
    def setUp(self):
        self.calibration = load_calibration(DEFAULT_CALIBRATION)

    def test_resolution_change_is_not_silently_scaled(self):
        with self.assertRaisesRegex(ValueError, 'not crop'):
            scaled_intrinsics(self.calibration, 1280, 720)

    def test_explicit_full_image_resize_preserves_projection(self):
        k, p = scaled_intrinsics(self.calibration, 960, 540, allow_resize=True)
        point = np.array([.4, -.2, 1.7])
        original = np.array(self.calibration.camera_matrix).reshape(3,3)@point
        resized = np.array(k).reshape(3,3)@point
        np.testing.assert_allclose(resized[:2]/resized[2], .5*original[:2]/original[2])
        np.testing.assert_allclose(np.array(p).reshape(3,4)[:,:3], np.array(k).reshape(3,3))

    def test_matching_resolution_is_unchanged(self):
        k,p = scaled_intrinsics(self.calibration,1920,1080)
        self.assertEqual(k,list(self.calibration.camera_matrix))
        self.assertEqual(p,list(self.calibration.projection_matrix))

    def test_default_needs_no_tf_and_no_assumed_resize(self):
        args=parse_args([])
        self.assertFalse(args.publish_static_tf)
        self.assertFalse(args.allow_intrinsics_resize)
        self.assertTrue(parse_args(['--publish-static-tf']).publish_static_tf)
