"""Unit-Tests für die Geometrie-Hilfsfunktionen."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from piscanner_perception.geometry import compute_lidar_points, rotate_to_world, vector_to_pano_uv


def test_compute_lidar_points_basic() -> None:
    ranges = np.array([1.0, 1.0], dtype=np.float32)
    x_local, z_local = compute_lidar_points(ranges, angle_min=-math.pi / 4, angle_increment=math.pi / 4)
    assert np.isclose(x_local[0], -math.cos(-math.pi / 4))
    assert np.isclose(z_local[0], math.sin(-math.pi / 4))
    assert np.isclose(z_local[1], 0.0)


def test_rotate_to_world_preserves_dimensions() -> None:
    x_local = np.array([0.0, -1.0], dtype=np.float32)
    z_local = np.array([1.0, 0.0], dtype=np.float32)
    points = rotate_to_world(x_local, z_local, motor_angle=math.pi / 2, lidar_offset_y=0.1, lidar_height=0.2)
    assert points.shape == (2, 3)
    assert np.allclose(points[:, 1], 0.1)


def test_vector_to_pano_uv_limits() -> None:
    vectors = np.array([[0.0, -1.0, 0.0], [0.0, -1.0, 1.0]], dtype=np.float32)
    u, v = vector_to_pano_uv(vectors, fov_deg=180.0, image_width=400, image_height=200)
    assert np.all((u >= 0) & (u < 400))
    assert np.all((v >= 0) & (v < 200))
