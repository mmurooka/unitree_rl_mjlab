#!/usr/bin/env python3
"""Record measured and commanded joints for offline comparison; publishes nothing."""
import argparse
import json
import math
import signal
import threading
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--network', default='lo')
    parser.add_argument('--domain-id', type=int, default=83)
    parser.add_argument('--environment', choices=('sim', 'real'), required=True)
    parser.add_argument('--duration', type=float, default=120)
    parser.add_argument('--rate', type=float, default=50)
    args = parser.parse_args()
    if not all(math.isfinite(x) and x > 0 for x in (args.duration, args.rate)):
        parser.error('duration and rate must be finite and positive')
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
    ChannelFactoryInitialize(args.domain_id, args.network)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    lock = threading.Lock()
    last = {}
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    with args.output.open('x') as stream:
        def write(kind, values):
            stamp = time.monotonic()
            with lock:
                if stamp-last.get(kind, -float('inf')) < 1/args.rate:
                    return
                last[kind] = stamp
                stream.write(json.dumps({'kind':kind, 'received_monotonic_s':stamp, **values})+'\n')
                stream.flush()

        def measured(msg):
            write('measured', {'tick_ms':msg.tick,
                              'tick_domain':'simulation' if args.environment=='sim' else 'robot_boot',
                              'quaternion_wxyz':list(msg.imu_state.quaternion),
                              'q':[float(x.q) for x in msg.motor_state[:29]],
                              'dq':[float(x.dq) for x in msg.motor_state[:29]]})

        def commanded(msg):
            write('commanded', {'q':[float(x.q) for x in msg.motor_cmd[:29]],
                               'kp':[float(x.kp) for x in msg.motor_cmd[:29]],
                               'kd':[float(x.kd) for x in msg.motor_cmd[:29]],
                               'tau':[float(x.tau) for x in msg.motor_cmd[:29]]})

        state = ChannelSubscriber('rt/lowstate', LowState_)
        command = ChannelSubscriber('rt/lowcmd', LowCmd_)
        state.Init(measured, 10)
        command.Init(commanded, 10)
        try:
            stop.wait(args.duration)
        finally:
            command.Close()
            state.Close()


if __name__ == '__main__':
    main()
