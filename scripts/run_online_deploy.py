#!/usr/bin/env python3
"""Launch the motion conversion worker and G1 controller together."""

import argparse
import signal
import subprocess
import sys
import termios
import time
from pathlib import Path


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--controller", default=str(root / "deploy/robots/g1/build/g1_ctrl")
    )
    parser.add_argument("--network", default="lo")
    parser.add_argument("--endpoint", default="ipc:///tmp/task_prompt_rl_motion.sock")
    parser.add_argument(
        "--deploy-endpoint", default="ipc:///tmp/task_prompt_rl_deploy.sock"
    )
    parser.add_argument("--debug-dir", default="/tmp/task_prompt_rl/received")
    parser.add_argument("--no-save-debug", action="store_true")
    args = parser.parse_args()
    worker = [
        sys.executable,
        str(root / "scripts/serve_motion_prompt.py"),
        "--endpoint",
        args.endpoint,
        "--deploy-endpoint",
        args.deploy_endpoint,
        "--debug-dir",
        args.debug_dir,
    ]
    if args.no_save_debug:
        worker.append("--no-save-debug")
    return run_processes([worker, [args.controller, "--network", args.network]], root)


def run_processes(commands, cwd):
    # The controller disables terminal echo for keyboard input. SIGTERM/SIGINT
    # can bypass its destructor, so the launcher must restore the original mode.
    terminal_fd = sys.stdin.fileno() if sys.stdin.isatty() else None
    terminal_settings = (
        termios.tcgetattr(terminal_fd) if terminal_fd is not None else None
    )
    processes = []
    try:
        for command in commands:
            processes.append(subprocess.Popen(command, cwd=cwd))
        while all(process.poll() is None for process in processes):
            time.sleep(0.2)
        return next(
            process.returncode
            for process in processes
            if process.returncode is not None
        )
    except KeyboardInterrupt:
        return 130
    finally:
        # Repeated Ctrl+C must not interrupt child cleanup or terminal restoration.
        previous_sigint = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            for process in processes:
                if process.poll() is None:
                    process.terminate()
            for process in processes:
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        finally:
            try:
                if terminal_settings is not None:
                    termios.tcsetattr(terminal_fd, termios.TCSANOW, terminal_settings)
            finally:
                signal.signal(signal.SIGINT, previous_sigint)


if __name__ == "__main__":
    raise SystemExit(main())
