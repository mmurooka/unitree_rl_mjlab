import sys
import time
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from measured_robot_state import MeasuredRobotState
from robot_frames import StateHistory


class MeasuredInputTest(unittest.TestCase):
    def reader(self, environment):
        reader = MeasuredRobotState.__new__(MeasuredRobotState)
        reader.environment = environment
        reader.clock_seconds = lambda: 1000.
        reader.history = StateHistory()
        reader.last_received = -float('inf')
        reader.last_tick = None
        reader.last_error = None
        return reader

    def message(self, tick):
        return SimpleNamespace(tick=tick,
            motor_state=[SimpleNamespace(q=.12) for _ in range(35)],
            imu_state=SimpleNamespace(quaternion=[1, 0, 0, 0]))

    def test_sim_uses_tick_real_uses_receipt_ros_time(self):
        for environment, stamp in [('sim', 2.), ('real', 1000.)]:
            reader = self.reader(environment)
            reader._receive(self.message(2000))
            np.testing.assert_allclose(reader.at(stamp).q, .12)
            self.assertEqual(reader.at(stamp).stamp, stamp)

    def test_repeated_tick_does_not_keep_frozen_state_fresh(self):
        reader = self.reader('sim')
        reader._receive(self.message(2000))
        reader.last_received = time.monotonic()-1
        reader._receive(self.message(2000))
        with self.assertRaises(ValueError): reader.at(2)

    def test_bad_quaternion_suppresses_even_previous_valid_sample(self):
        reader = self.reader('sim')
        reader._receive(self.message(2000))
        bad = self.message(2001)
        bad.imu_state.quaternion = [0]*4
        reader._receive(bad)
        with self.assertRaises(ValueError): reader.at(2)


if __name__ == '__main__': unittest.main()
