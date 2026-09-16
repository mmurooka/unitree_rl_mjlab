#!/usr/bin/env python3
"""Launch the motion conversion worker and G1 controller together."""
import argparse
from pathlib import Path
import subprocess
import sys
import time


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", default=str(root / "deploy/robots/g1/build/g1_ctrl"))
    parser.add_argument("--network", default="lo")
    parser.add_argument("--endpoint", default="ipc:///tmp/task_prompt_rl_motion.sock")
    parser.add_argument("--deploy-endpoint", default="ipc:///tmp/task_prompt_rl_deploy.sock")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--debug-dir", default="/tmp/task_prompt_rl/received")
    parser.add_argument("--no-save-debug", action="store_true")
    args = parser.parse_args()
    worker = [sys.executable, str(root / "scripts/serve_motion_prompt.py"),
              "--endpoint", args.endpoint, "--deploy-endpoint", args.deploy_endpoint,
              "--device", args.device, "--debug-dir", args.debug_dir]
    if args.no_save_debug:
        worker.append("--no-save-debug")
    processes = []
    try:
        processes.append(subprocess.Popen(worker, cwd=root))
        processes.append(subprocess.Popen([args.controller, "--network", args.network], cwd=root))
        while all(process.poll() is None for process in processes):
            time.sleep(0.2)
        return next(process.returncode for process in processes if process.returncode is not None)
    except KeyboardInterrupt:
        return 130
    finally:
        for process in processes:
            if process.poll() is None:
                process.terminate()
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


if __name__ == "__main__":
    raise SystemExit(main())
