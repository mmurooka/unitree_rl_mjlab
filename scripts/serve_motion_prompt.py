#!/usr/bin/env python3
"""ZMQ MotionPrompt receiver and persistent offline-equivalent converter."""

from __future__ import annotations

import argparse
import io
import json
import tempfile
import threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
import zmq
from motion_prompt_npz import read_motion_prompt_fps


def validate_prompt(payload: bytes, dofs: int = 29):
    with np.load(io.BytesIO(payload), allow_pickle=False) as data:
        read_motion_prompt_fps(data)
        if set(data.files) != {"format_version", "fps", "q_ref", "foot_contact"}:
            raise ValueError(
                "Expected only format_version, fps, q_ref and foot_contact"
            )
        if data["format_version"].shape != () or data["format_version"].item() != 1:
            raise ValueError("Unsupported MotionPrompt format_version")
        q, contacts = data["q_ref"], data["foot_contact"]
        if q.ndim != 2 or q.shape[1] != 7 + dofs or len(q) < 4:
            raise ValueError(f"q_ref must have shape [N>=4, {7 + dofs}]")
        if contacts.shape != (len(q), 2):
            raise ValueError("foot_contact must have shape [N, 2]")
        if not np.isfinite(q).all() or not np.isfinite(contacts).all():
            raise ValueError("Non-finite MotionPrompt values")
        if not np.allclose(np.linalg.norm(q[:, 3:7], axis=1), 1, atol=1e-3):
            raise ValueError("q_ref contains a non-unit quaternion")


def deploy_request(endpoint: str, message: str, timeout_ms: int = 10000) -> str:
    # Each socket belongs to this calling thread; failed requests are not resent.
    with zmq.Context() as context, context.socket(zmq.REQ) as socket:
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.IMMEDIATE, 1)
        socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
        socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        socket.connect(endpoint)
        try:
            socket.send_string(message)
            return socket.recv_string()
        except zmq.Again as error:
            raise TimeoutError("Deploy timeout; outcome unknown, no retry") from error


class MotionService:
    def __init__(
        self,
        converter,
        deploy_endpoint,
        *,
        save_debug=True,
        debug_dir="/tmp/task_prompt_rl/received",
        dofs=29,
    ):
        self.converter = converter
        self.deploy_endpoint = deploy_endpoint
        self.save_debug = save_debug
        self.debug_dir = Path(debug_dir)
        self.dofs = dofs

    def execute(self, input_path: Path, output_path: Path):
        try:
            status = deploy_request(self.deploy_endpoint, "STATUS")
            if status != "READY":
                return {
                    "status": "BUSY" if status == "BUSY" else "ERROR",
                    "message": status,
                }
            validate_prompt(input_path.read_bytes(), self.dofs)
            self.converter.convert(str(input_path), str(output_path))
            status = deploy_request(self.deploy_endpoint, "LOAD\n" + str(output_path))
            return {
                "status": "STARTED"
                if status == "STARTED"
                else "BUSY"
                if status == "BUSY"
                else "ERROR",
                "message": status,
            }
        except Exception as error:
            return {"status": "ERROR", "message": str(error)}

    def serve(self, endpoint, stop: threading.Event | None = None):
        stop = stop or threading.Event()
        if self.save_debug:
            self.debug_dir.mkdir(parents=True, exist_ok=True)
        # ROUTER permits BUSY replies while the single converter is occupied.
        # The client still sends exactly one NPZ as a standard REQ request.
        with (
            zmq.Context() as context,
            context.socket(zmq.ROUTER) as socket,
            ThreadPoolExecutor(max_workers=1) as executor,
        ):
            socket.setsockopt(zmq.LINGER, 0)
            socket.setsockopt(zmq.MAXMSGSIZE, 256 * 1024 * 1024)
            socket.bind(endpoint)
            pending = None
            print(f"MotionPrompt receiver ready: {endpoint}", flush=True)
            try:
                while not stop.is_set():
                    if pending is not None and pending[0].done():
                        future, address, temporary = pending
                        socket.send_multipart(
                            address + [json.dumps(future.result()).encode()]
                        )
                        if temporary is not None:
                            temporary.cleanup()
                        pending = None
                    if not socket.poll(20):
                        continue
                    parts = socket.recv_multipart()
                    if len(parts) != 3 or parts[1] != b"":
                        continue
                    address, payload = parts[:2], parts[2]
                    temporary = None
                    if self.save_debug:
                        folder = Path(
                            tempfile.mkdtemp(prefix="motion_", dir=self.debug_dir)
                        )
                    else:
                        temporary = tempfile.TemporaryDirectory(
                            prefix="motion_prompt_", dir="/tmp"
                        )
                        folder = Path(temporary.name)
                    input_path, output_path = (
                        folder / "motion_prompt.npz",
                        folder / "policy_input.npz",
                    )
                    input_path.write_bytes(payload)
                    if pending is not None:
                        socket.send_multipart(address + [b'{"status":"BUSY"}'])
                        if temporary is not None:
                            temporary.cleanup()
                        continue
                    pending = (
                        executor.submit(self.execute, input_path, output_path),
                        address,
                        temporary,
                    )
            finally:
                if pending is not None:
                    pending[0].result()
                    if pending[2] is not None:
                        pending[2].cleanup()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="ipc:///tmp/task_prompt_rl_motion.sock")
    parser.add_argument(
        "--deploy-endpoint", default="ipc:///tmp/task_prompt_rl_deploy.sock"
    )
    parser.add_argument("--debug-dir", default="/tmp/task_prompt_rl/received")
    parser.add_argument("--no-save-debug", action="store_true")
    args = parser.parse_args()
    from convert_motion_prompt_npz import MotionPromptConverter

    service = MotionService(
        MotionPromptConverter("g1"),
        args.deploy_endpoint,
        save_debug=not args.no_save_debug,
        debug_dir=args.debug_dir,
    )
    try:
        service.serve(args.endpoint)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
