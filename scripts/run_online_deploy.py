#!/usr/bin/env python3
"""Launch the motion conversion worker and G1 controller together."""

import argparse
import os
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
    parser.add_argument(
        "--navigation-localization-source",
        choices=("glim", "simulator"),
        required=True,
        help="override the selected Navigation policy localization source",
    )
    parser.add_argument("--environment", choices=("sim", "real"), default="real")
    parser.add_argument("--domain-id", type=int, default=0)
    parser.add_argument("--glim-config", type=Path)
    parser.add_argument("--sensor-model", type=Path)
    parser.add_argument("--endpoint", default="ipc:///tmp/task_prompt_rl_motion.sock")
    parser.add_argument(
        "--deploy-endpoint", default="ipc:///tmp/task_prompt_rl_deploy.sock"
    )
    parser.add_argument("--debug-dir", default="/tmp/task_prompt_rl/received")
    parser.add_argument("--no-save-debug", action="store_true")
    args = parser.parse_args()
    if args.environment == "real" and args.navigation_localization_source == "simulator":
        parser.error("simulator ground truth cannot be used on real hardware")
    if args.navigation_localization_source == "glim" and args.glim_config is None:
        parser.error("--glim-config must match the externally started GLIM config_path")
    if args.domain_id < 0:
        parser.error("--domain-id must be nonnegative")
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
    controller = [args.controller, "--network", args.network,
                  "--environment", args.environment, "--domain-id", str(args.domain_id)]
    if args.glim_config is not None:
        controller.extend(["--glim-config", str(args.glim_config.resolve())])
    if args.sensor_model is not None:
        controller.extend(["--sensor-model", str(args.sensor_model.resolve())])
    if args.navigation_localization_source is not None:
        controller.extend(
            [
                "--navigation-localization-source",
                args.navigation_localization_source,
            ]
        )
    return run_processes([worker, controller], root)


def run_processes(commands, cwd):
    # Every child owns a process group. Terminate the group even when its leader
    # exited first, so workers/grandchildren cannot outlive the launcher.
    terminal_fd = sys.stdin.fileno() if sys.stdin.isatty() else None
    terminal_settings = termios.tcgetattr(terminal_fd) if terminal_fd is not None else None
    processes = []

    class Shutdown(Exception):
        def __init__(self, signum): self.signum = signum

    def stop(signum, _frame):
        raise Shutdown(signum)

    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        for command in commands:
            processes.append(subprocess.Popen(command, cwd=cwd, start_new_session=True))
        while all(process.poll() is None for process in processes):
            time.sleep(0.2)
        return next(process.returncode for process in processes if process.returncode is not None)
    except Shutdown as error:
        return 128 + error.signum
    finally:
        for sig in previous: signal.signal(sig, signal.SIG_IGN)
        try:
            for process in processes:
                try: os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError: pass
            deadline = time.monotonic() + 5
            for process in processes:
                try: process.wait(timeout=max(0, deadline-time.monotonic()))
                except subprocess.TimeoutExpired: pass
            for process in processes:
                try: os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError: pass
                process.wait()
        finally:
            try:
                if terminal_settings is not None:
                    termios.tcsetattr(terminal_fd, termios.TCSANOW, terminal_settings)
            finally:
                for sig, handler in previous.items(): signal.signal(sig, handler)


if __name__ == "__main__":
    raise SystemExit(main())
