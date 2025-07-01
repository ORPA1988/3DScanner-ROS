#!/usr/bin/env python3
import sys
from pathlib import Path

# Allow running the script directly from the repository root
sys.path.append(str(Path(__file__).resolve().parents[1]))

from scanner.lidar import STL27L  # noqa: E402
from scanner.stepper import Stepper  # noqa: E402
from scanner.scanner import Scanner  # noqa: E402
from scanner.pointcloud import save_ply  # noqa: E402


def main(output_file: str) -> None:
    lidar = STL27L()
    stepper = Stepper(step_pin=21, dir_pin=20, enable_pin=16)
    scanner = Scanner(lidar, stepper)
    try:
        points = scanner.scan(rotations=1)
        save_ply(points, output_file)
    finally:
        stepper.cleanup()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: run_scan.py output.ply")
        sys.exit(1)
    main(sys.argv[1])
