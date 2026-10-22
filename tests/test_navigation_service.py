"""Real local IPC protocol, no robot interfaces."""
import subprocess
import json
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
                    def request(cmd):
                        sock.send_string(cmd)
                        return sock.recv_string()
                    self.assertEqual(request('PROGRESS'), 'IDLE 0')
                    plan = json.loads(request('PLAN -1.2 0 0'))
                    self.assertEqual(len(plan['poses']), 51)
                    self.assertEqual(plan['poses'][0], [0, 0, 0])
                    self.assertAlmostEqual(plan['poses'][-1][0], -1.2, places=6)
                    self.assertTrue(all(abs(p[1])<1e-6 and abs(p[2])<1e-6 for p in plan['poses']))
                    self.assertEqual(request('PROGRESS'), 'IDLE 0', 'planning must not queue a goal')
                    turn = json.loads(request('PLAN 0 0 1.0'))
                    self.assertAlmostEqual(turn['poses'][25][2], .5, places=6)
                    self.assertTrue(request('PLAN nan 0 0').startswith('ERROR'))
                    view = json.loads(request('VIEW'))
                    self.assertEqual(view['root']['source'], 'simulator')
                    self.assertIsNone(view['path'])
                    self.assertEqual(request('PROGRESS'), 'IDLE 0')
                    last_id = 0
                    for cmd, state in (
                        ('GOAL_AT simulation 12.5 1 2 .3', 'REACHED'),
                        ('GOAL 7 2 .3', 'RUNNING'),
                        ('GOAL 8 2 .3', 'ERROR'),
                        ('GOAL 9 2 .3', 'CANCELLED'),
                    ):
                        response = request(cmd).split()
                        self.assertEqual(response[0], 'STARTED')
                        goal_id = int(response[1])
                        self.assertGreater(goal_id, last_id)
                        last_id = goal_id
                        # stdout is emitted after the terminal status is set.
                        line = process.stdout.readline().split()
                        if cmd.startswith('GOAL_AT'):
                            self.assertEqual(line[:2], ['simulation', '12.5'])
                        progress = request('PROGRESS').split()
                        self.assertEqual(progress[:2], [state, str(goal_id)])
                        view = json.loads(request('VIEW'))
                        self.assertEqual(view['path']['goal_id'], str(goal_id))
                        self.assertEqual(view['goal_id'], str(goal_id))
                        self.assertEqual(view['progress'], state)
                    for cmd in ('GOAL_AT unknown 12 1 2 3', 'GOAL_AT ros -1 1 2 3',
                                'GOAL_AT ros nan 1 2 3', 'GOAL_AT ros 12 1 2',
                                'GOAL 1 2 3 extra', 'SEARCH .1 1.57'):
                        self.assertTrue(request(cmd).startswith('ERROR'))
            finally:
                process.terminate()
                process.wait(timeout=5)
                process.stdout.close()
