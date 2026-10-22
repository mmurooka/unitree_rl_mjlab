"""Real local IPC protocol, no robot interfaces."""
import subprocess
import tempfile
import unittest
from pathlib import Path
import zmq

ROOT = Path(__file__).resolve().parents[1]


class NavigationProtocolTest(unittest.TestCase):
    def test_observation_timestamp_and_legacy_goal_protocol(self):
        with tempfile.TemporaryDirectory() as directory:
            exe = str(Path(directory) / 'probe')
            subprocess.run(['g++', '-std=c++17', '-pthread',
                            '-I' + str(ROOT / 'deploy/robots/g1/include'),
                            str(ROOT / 'tests/navigation_service_probe.cpp'),
                            str(ROOT / 'deploy/robots/g1/src/NavigationGoalService.cpp'),
                            '-lyaml-cpp', '-lzmq', '-o', exe], check=True, capture_output=True)
            endpoint = 'ipc://' + directory + '/nav.sock'
            process = subprocess.Popen([exe, endpoint], stdout=subprocess.PIPE, text=True)
            try:
                with zmq.Context() as context, context.socket(zmq.REQ) as sock:
                    sock.setsockopt(zmq.LINGER, 0)
                    sock.setsockopt(zmq.RCVTIMEO, 3000)
                    sock.connect(endpoint)
                    for cmd in ('GOAL_AT simulation 12.5 1 2 .3', 'GOAL 1 2 .3', 'SEARCH .1 1.57'):
                        sock.send_string(cmd)
                        self.assertEqual(sock.recv_string(), 'STARTED')
                    first = process.stdout.readline().split()
                    self.assertEqual(first[:2], ['simulation', '12.5'])
                    for cmd in ('GOAL_AT unknown 12 1 2 3', 'GOAL_AT ros -1 1 2 3',
                                'GOAL_AT ros nan 1 2 3', 'GOAL_AT ros 12 1 2', 'GOAL 1 2 3 extra'):
                        sock.send_string(cmd)
                        self.assertTrue(sock.recv_string().startswith('ERROR'))
            finally:
                process.terminate()
                process.wait(timeout=5)
                process.stdout.close()
