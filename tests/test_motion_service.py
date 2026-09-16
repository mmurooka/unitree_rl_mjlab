import io
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np
import zmq

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from serve_motion_prompt import MotionService, validate_prompt


def payload():
    buffer = io.BytesIO()
    q = np.zeros((5, 36)); q[:, 6] = 1
    np.savez(buffer, format_version=np.asarray(1, dtype=np.int64),
             q_ref=q, foot_contact=np.ones((5, 2)))
    return buffer.getvalue()


class FakeConverter:
    def __init__(self):
        self.entered = threading.Event()
        self.proceed = threading.Event()
        self.calls = 0

    def convert(self, source, target):
        self.calls += 1
        self.entered.set()
        if not self.proceed.wait(5):
            raise TimeoutError('test converter blocked')
        Path(target).write_bytes(Path(source).read_bytes())


class ServiceTests(unittest.TestCase):
    def test_validation(self):
        validate_prompt(payload())
        with self.assertRaises(Exception):
            validate_prompt(b'bad npz')
        with np.load(io.BytesIO(payload())) as d:
            q = d['q_ref']; q[0, 7] = np.nan
        b = io.BytesIO()
        np.savez(b, format_version=1, q_ref=q, foot_contact=np.ones((5, 2)))
        with self.assertRaisesRegex(ValueError, 'Non-finite'):
            validate_prompt(b.getvalue())

    def check_service(self, save_debug):
        with tempfile.TemporaryDirectory() as folder, zmq.Context() as context:
            endpoint = 'ipc://' + folder + '/service.sock'
            converter = FakeConverter()
            service = MotionService(converter, 'unused', save_debug=save_debug,
                                    debug_dir=folder + '/debug')
            stop = threading.Event()
            calls = []
            def deploy(endpoint, command):
                calls.append(command)
                if command == 'STATUS': return 'READY'
                self.assertTrue(Path(command.split('\n')[1]).exists())
                return 'STARTED'
            with patch('serve_motion_prompt.deploy_request', deploy):
                thread = threading.Thread(target=service.serve, args=(endpoint, stop))
                thread.start()
                try:
                    with context.socket(zmq.REQ) as first, context.socket(zmq.REQ) as second:
                        for sock in (first, second):
                            sock.setsockopt(zmq.LINGER, 0)
                            sock.setsockopt(zmq.RCVTIMEO, 5000)
                            sock.connect(endpoint)
                        first.send(payload())
                        self.assertTrue(converter.entered.wait(3))
                        second.send(payload())
                        self.assertEqual(second.recv_json()['status'], 'BUSY')
                        converter.proceed.set()
                        self.assertEqual(first.recv_json()['status'], 'STARTED')
                        time.sleep(.05)
                        output = Path(calls[-1].split('\n')[1])
                        self.assertEqual(output.exists(), save_debug)
                        self.assertEqual(converter.calls, 1)
                        if save_debug:
                            inputs = list(Path(folder).rglob('motion_prompt.npz'))
                            self.assertEqual(len(inputs), 2)
                            self.assertTrue(all(p.read_bytes() == payload() for p in inputs))
                        else:
                            self.assertFalse(Path(folder, 'debug').exists())
                        first.send(b'invalid')
                        self.assertEqual(first.recv_json()['status'], 'ERROR')
                        self.assertEqual(converter.calls, 1)
                finally:
                    stop.set(); converter.proceed.set(); thread.join(5)
                    self.assertFalse(thread.is_alive())

    def test_debug_and_busy(self): self.check_service(True)
    def test_no_debug_cleanup(self): self.check_service(False)

    def test_deploy_busy(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder, 'input.npz'); source.write_bytes(payload())
            c = FakeConverter()
            s = MotionService(c, 'unused')
            with patch('serve_motion_prompt.deploy_request', return_value='BUSY'):
                self.assertEqual(s.execute(source, Path(folder, 'output.npz'))['status'], 'BUSY')
            self.assertEqual(c.calls, 0)


if __name__ == '__main__': unittest.main()
