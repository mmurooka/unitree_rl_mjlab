"""Known rigid map offsets must cancel; trajectory distortion must remain."""
import csv
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from evaluate_navigation_frames import evaluate


class FrameEvaluationTest(unittest.TestCase):
    def comparison(self, distortion=0):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            poses = folder / 'poses.jsonl'
            rows = []
            # Known world robot poses: (0,0,0), (1,0,0), (1,1,pi/2).
            for stamp, relative in ((.95, [1, 0, 0, 0, 0, 0, 1]),
                                    (1., [1, 0, 0, 0, 0, 0, 1]),
                                    (1.05, [0, 0, 0, 0, 0, 0, 1]),
                                    (1.1, [-1, 0, 0, 0, 0, -math.sqrt(.5), math.sqrt(.5)])):
                rows.append(dict(topic='/evaluation/frame_pose_list', data=dict(
                    stamp=stamp, source='evaluation_ground_truth', reference_frame='root_yaw',
                    frame_pose_list=[dict(frame_name='table', rel_pose=relative)])))
            poses.write_text(''.join(json.dumps(row)+'\n' for row in rows))
            path = folder / 'nav.csv'
            with path.open('w') as stream:
                writer = csv.writer(stream)
                writer.writerow(['source', 'localization_available', 'localization_stamp_s',
                                 'slam_pose_x_m', 'slam_pose_y_m', 'slam_pose_theta_rad'])
                # Same path in a map rotated +90 degrees and shifted (4,3).
                writer.writerows([['glim', 1, 1., 4, 3, math.pi/2],
                                  ['glim', 1, 1.05, 4, 4, math.pi/2],
                                  ['glim', 1, 1.1, 3+distortion, 4, math.pi]])
            return evaluate(poses, [path], 'table', [1, 0, 0])

    def test_map_offset_is_aligned_once(self):
        result = self.comparison()
        self.assertEqual(result['matched_samples'], 3)
        self.assertLess(result['position_max_m'], 1e-12)
        self.assertLess(result['yaw_max_abs_rad'], 1e-12)

    def test_distortion_is_not_hidden_by_alignment(self):
        result = self.comparison(.5)
        self.assertAlmostEqual(result['position_max_m'], .5)
        self.assertAlmostEqual(result['position_rmse_m'], .5/math.sqrt(3))
