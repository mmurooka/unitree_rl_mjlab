"""Exercise terminal restoration with a real child process and pseudo-terminal."""

import os
import pty
import signal
import subprocess
import sys
import tempfile
import termios
import time
import unittest
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


class LauncherTerminalTest(unittest.TestCase):
    def check_terminal(self, mode):
        master, slave = pty.openpty()
        process = None
        try:
            before = termios.tcgetattr(slave)
            with tempfile.TemporaryDirectory() as directory:
                ready = Path(directory) / "ready"
                child = f"""
import os, signal, termios, time
from pathlib import Path
settings = termios.tcgetattr(0)
settings[3] &= ~(termios.ECHO | termios.ICANON)
termios.tcsetattr(0, termios.TCSANOW, settings)
if {mode!r} == 'kill':
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
Path({str(ready)!r}).write_text(str(os.getpid()))
time.sleep(0.2 if {mode!r} == 'exit' else 60)
raise SystemExit(7)
"""
                launcher = f"""
import sys
sys.path.insert(0, {str(SCRIPTS)!r})
from run_online_deploy import run_processes
raise SystemExit(run_processes([[sys.executable, '-c', {child!r}]], {directory!r}))
"""
                process = subprocess.Popen(
                    [sys.executable, "-c", launcher],
                    stdin=slave,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    start_new_session=True,
                )
                deadline = time.monotonic() + 5
                while not ready.exists():
                    self.assertIsNone(
                        process.poll(), "launcher exited before child readiness"
                    )
                    self.assertLess(
                        time.monotonic(), deadline, "child readiness timeout"
                    )
                    time.sleep(0.01)
                if mode != "exit":
                    self.assertFalse(termios.tcgetattr(slave)[3] & termios.ECHO)
                    process.send_signal(signal.SIGINT)
                    if mode == "kill":
                        time.sleep(0.2)
                        process.send_signal(signal.SIGINT)
                _, stderr = process.communicate(timeout=10)
                self.assertEqual(
                    process.returncode, 7 if mode == "exit" else 130, stderr.decode()
                )
                self.assertEqual(termios.tcgetattr(slave), before)
                with self.assertRaises(ProcessLookupError):
                    os.kill(int(ready.read_text()), 0)
        finally:
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            os.close(master)
            os.close(slave)

    def test_child_exit_restores_terminal(self):
        self.check_terminal("exit")

    def test_ctrl_c_restores_terminal(self):
        self.check_terminal("interrupt")

    def test_forced_kill_and_repeated_ctrl_c_restore_terminal(self):
        self.check_terminal("kill")

    def test_without_terminal(self):
        launcher = f"""
import sys
sys.path.insert(0, {str(SCRIPTS)!r})
from run_online_deploy import run_processes
raise SystemExit(run_processes([[sys.executable, '-c', 'pass']], {str(SCRIPTS)!r}))
"""
        result = subprocess.run(
            [sys.executable, "-c", launcher],
            stdin=subprocess.DEVNULL,
            capture_output=True,
            timeout=5,
        )
        self.assertEqual(result.returncode, 0, result.stderr.decode())
