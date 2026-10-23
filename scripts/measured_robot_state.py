"""Read measured SDK state for model-based perception, without motor commands."""
from __future__ import annotations

import time

from robot_frames import StateHistory


class MeasuredRobotState:
    def __init__(self, environment, clock_seconds, network=None, domain_id=0):
        if environment not in ('sim', 'real'):
            raise ValueError('environment must be sim or real')
        # Keep geometry/tests independent of the SDK and its DDS dependencies.
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        self.environment = environment
        self.clock_seconds = clock_seconds
        self.history = StateHistory()
        self.last_received = -float('inf')
        self.last_tick = None
        self.last_error = None
        ChannelFactoryInitialize(domain_id, network)
        self.subscriber = ChannelSubscriber('rt/lowstate', LowState_)
        self.subscriber.Init(self._receive, 10)

    def _receive(self, message):
        # Sim tick is milliseconds of simulation time. On hardware the tick is
        # a boot clock, not ROS time; use receipt ROS time, explicitly an
        # approximation until hardware capture timestamps are available.
        if message.tick == self.last_tick:
            return
        stamp = message.tick * .001 if self.environment == 'sim' else self.clock_seconds()
        try:
            self.history.add(stamp, [s.q for s in message.motor_state[:29]], message.imu_state.quaternion)
        except ValueError as error:
            self.last_error = str(error)
            return
        self.last_tick = message.tick
        self.last_received = time.monotonic()
        self.last_error = None

    def at(self, stamp):
        if time.monotonic() - self.last_received > .5:
            raise ValueError('measured rt/lowstate is unavailable or stale')
        if self.last_error:
            raise ValueError(self.last_error)
        return self.history.at(stamp)

    def latest(self):
        """Latest validated sample, with the same freshness guard as at()."""
        with self.history.lock:
            if not self.history.samples:
                raise ValueError('waiting for measured robot state')
            stamp = self.history.samples[-1].stamp
        return self.at(stamp)

    def close(self):
        self.subscriber.Close()
