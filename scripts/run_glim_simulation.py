#!/usr/bin/env python3
"""Run MuJoCo, GLIM, and marker detection through the real-robot ROS path."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from run_online_deploy import run_processes


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SIMULATOR = ROOT / "simulate" / "build" / "unitree_mujoco"
DEFAULT_OVERRIDES = ROOT / "SLAM" / "GLIM_config_sim"
DEFAULT_SCENE = (
    ROOT
    / "src"
    / "assets"
    / "robots"
    / "unitree_g1"
    / "xmls"
    / "scene_g1_table_box_glim.xml"
)


def find_glim_base_config(ros2: str) -> Path:
    try:
        result = subprocess.run(
            [ros2, "pkg", "prefix", "glim"],
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError) as error:
        raise RuntimeError(
            "GLIM was not found; source ROS 2 and install the glim package"
        ) from error
    config = Path(result.stdout.strip()) / "share" / "glim" / "config"
    if not (config / "config.json").is_file():
        raise RuntimeError(f"GLIM base configuration not found: {config}")
    return config


def prepare_glim_config(base: Path, overrides: Path, destination: Path) -> None:
    required_overrides = ("config_ros.json", "config_sensors.json")
    if not (base / "config.json").is_file():
        raise ValueError(f"GLIM base configuration is incomplete: {base}")
    missing = [name for name in required_overrides if not (overrides / name).is_file()]
    if missing:
        raise ValueError(
            f"GLIM simulation overrides are missing: {', '.join(missing)}"
        )

    shutil.copytree(base, destination, dirs_exist_ok=True)
    for name in required_overrides:
        shutil.copy2(overrides / name, destination / name)


def build_commands(
    args: argparse.Namespace,
    ros2: str,
    merged_config: Path,
) -> list[list[str]]:
    simulator = [
        str(args.simulator),
        "--glim",
        "--network",
        args.network,
        "--domain_id",
        str(args.domain_id),
    ]
    if args.scene is not None:
        simulator.extend(["--scene", str(args.scene)])

    commands = [
        [
            ros2,
            "run",
            "glim_ros",
            "glim_rosnode",
            "--ros-args",
            "-p",
            f"config_path:={merged_config}",
        ],
        simulator,
    ]
    if not args.no_aruco:
        marker = [
            sys.executable,
            str(ROOT / "aruco" / "frame_pose_publisher.py"),
            "--glim",
        ]
        if args.aruco_config is not None:
            marker.extend(["--config", str(args.aruco_config)])
        commands.append(marker)
    if args.with_deploy:
        commands.append(
            [
                sys.executable,
                str(ROOT / "scripts" / "run_online_deploy.py"),
                "--network",
                args.network,
                "--navigation-localization-source",
                "glim",
            ]
        )
    return commands


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--simulator", type=Path, default=DEFAULT_SIMULATOR)
    parser.add_argument(
        "--glim-base-config",
        type=Path,
        help="base GLIM config directory (default: discover from ROS package)",
    )
    parser.add_argument(
        "--glim-overrides",
        type=Path,
        default=DEFAULT_OVERRIDES,
        help="simulation-specific config_ros.json/config_sensors.json directory",
    )
    parser.add_argument("--network", default="lo")
    parser.add_argument("--domain-id", type=int, default=0)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--no-aruco", action="store_true")
    parser.add_argument("--aruco-config", type=Path)
    parser.add_argument(
        "--with-deploy",
        action="store_true",
        help="also start run_online_deploy.py with GLIM forced",
    )
    args = parser.parse_args(argv)
    if not args.simulator.is_file():
        parser.error(
            f"simulator not found: {args.simulator}; build simulate/ first"
        )
    if args.scene is not None and not args.scene.is_file():
        parser.error(f"scene not found: {args.scene}")
    if not args.network:
        parser.error("--network must not be empty")
    if args.domain_id < 0:
        parser.error("--domain-id must be non-negative")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    ros2 = shutil.which("ros2")
    if ros2 is None:
        print("error: ros2 not found; source the ROS 2 setup first", file=sys.stderr)
        return 2

    try:
        base_config = (
            args.glim_base_config
            if args.glim_base_config is not None
            else find_glim_base_config(ros2)
        )
        with tempfile.TemporaryDirectory(prefix="unitree_glim_sim_") as directory:
            merged_config = Path(directory) / "config"
            prepare_glim_config(
                base_config.resolve(),
                args.glim_overrides.resolve(),
                merged_config,
            )
            print(
                "Starting GLIM simulation mode: "
                "MID-360 -> GLIM -> /glim_ros/odom -> Navigation",
                flush=True,
            )
            return run_processes(
                build_commands(args, ros2, merged_config),
                ROOT,
            )
    except (OSError, RuntimeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
