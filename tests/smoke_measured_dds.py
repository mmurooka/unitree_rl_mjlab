"""Opt-in DDS loopback check. Publishes only synthetic LowState on domain 83.

Never sends motor commands. Run separately from controller/simulator tests.
"""
from pathlib import Path
import sys
import time
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from measured_robot_state import MeasuredRobotState
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_

reader = MeasuredRobotState('sim', time.time, network='lo', domain_id=83)
writer = ChannelPublisher('rt/lowstate', LowState_)
writer.Init()
try:
    msg = unitree_hg_msg_dds__LowState_()
    msg.imu_state.quaternion = [1., 0., 0., 0.]
    msg.motor_state[12].q = .3
    for i in range(60):
        msg.tick = 1000+i*5
        writer.Write(msg)
        time.sleep(.005)
    actual = reader.at(msg.tick*.001)
    np.testing.assert_allclose(actual.q[12], .3, atol=1e-6)
    print('DDS measured state: OK; tick stamp',actual.stamp)
finally:
    writer.Close()
    reader.close()
