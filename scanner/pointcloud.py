from dataclasses import dataclass
from typing import List


@dataclass
class Point:
    x: float
    y: float
    z: float


def save_ply(points: List[Point], path: str) -> None:
    """Write points to a PLY file."""
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("ply\n")
        fh.write("format ascii 1.0\n")
        fh.write(f"element vertex {len(points)}\n")
        fh.write("property float x\n")
        fh.write("property float y\n")
        fh.write("property float z\n")
        fh.write("end_header\n")
        for p in points:
            fh.write(f"{p.x} {p.y} {p.z}\n")
