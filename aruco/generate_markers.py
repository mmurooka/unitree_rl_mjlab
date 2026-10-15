#!/usr/bin/env python3
"""Generate the marker PNG assets referenced by the G1 MuJoCo scene."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "simulate" / "config.yaml"
DEFAULT_OUTPUT = ROOT / "src" / "assets" / "robots" / "unitree_g1" / "xmls" / "assets"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--marker-pixels", type=int, default=480)
    parser.add_argument("--quiet-zone-pixels", type=int, default=60)
    args = parser.parse_args()

    with args.config.open("r", encoding="utf-8") as stream:
        frame_config = yaml.safe_load(stream)["frame_pose_list"]
    aruco_config = frame_config["aruco"]
    dictionary_name = aruco_config["dictionary"]
    dictionary_id = getattr(cv2.aruco, dictionary_name, None)
    if dictionary_id is None:
        raise ValueError(f"unknown OpenCV ArUco dictionary: {dictionary_name}")
    if args.marker_pixels <= 0 or args.quiet_zone_pixels <= 0:
        raise ValueError("pixel sizes must be positive")

    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    canvas_size = args.marker_pixels + 2 * args.quiet_zone_pixels
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for raw_id in aruco_config["markers"]:
        marker_id = int(raw_id)
        if hasattr(cv2.aruco, "generateImageMarker"):
            marker = cv2.aruco.generateImageMarker(
                dictionary, marker_id, args.marker_pixels
            )
        else:
            marker = cv2.aruco.drawMarker(
                dictionary, marker_id, args.marker_pixels
            )
        canvas = np.full((canvas_size, canvas_size), 255, dtype=np.uint8)
        start = args.quiet_zone_pixels
        canvas[start : start + args.marker_pixels, start : start + args.marker_pixels] = marker
        output = args.output_dir / f"aruco_marker_{marker_id}.png"
        if not cv2.imwrite(str(output), canvas):
            raise OSError(f"failed to write {output}")
        print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
