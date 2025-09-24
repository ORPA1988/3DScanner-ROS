"""Geometrie-Hilfsfunktionen für die Punktwolkenberechnung."""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np


def compute_lidar_points(ranges: np.ndarray, angle_min: float, angle_increment: float) -> Tuple[np.ndarray, np.ndarray]:
    """Berechnet die XZ-Koordinaten im LiDAR-Koordinatensystem."""

    num_measurements = ranges.shape[0]
    angles = angle_min + np.arange(num_measurements) * angle_increment
    x_local = -np.cos(angles) * ranges
    z_local = np.sin(angles) * ranges
    return x_local, z_local


def rotate_to_world(
    x_local: np.ndarray,
    z_local: np.ndarray,
    motor_angle: float,
    lidar_offset_y: float,
    lidar_height: float,
) -> np.ndarray:
    """Transformiert lokale LiDAR-Koordinaten in das Weltkoordinatensystem."""

    cos_angle = math.cos(motor_angle)
    sin_angle = math.sin(motor_angle)
    x_world = cos_angle * x_local + sin_angle * z_local
    z_world = -sin_angle * x_local + cos_angle * z_local
    y_world = np.full_like(x_world, lidar_offset_y)
    z_world = z_world + lidar_height
    points = np.stack([x_world, y_world, z_world], axis=-1)
    return points


def vector_to_pano_uv(vector: np.ndarray, fov_deg: float, image_width: int, image_height: int) -> Tuple[np.ndarray, np.ndarray]:
    """Projiziert 3D-Richtungen auf eine equirektangulare Panorama-Textur."""

    norms = np.linalg.norm(vector, axis=-1, keepdims=True)
    norms[norms == 0.0] = 1e-6
    normalized = vector / norms
    max_angle = math.radians(fov_deg / 2.0)

    azimuth = np.arctan2(normalized[:, 0], -normalized[:, 1])
    elevation = np.arcsin(np.clip(normalized[:, 2], -1.0, 1.0))

    u = (azimuth + max_angle) / (2 * max_angle)
    v = (elevation + max_angle) / (2 * max_angle)

    u = np.clip(u, 0.0, 1.0)
    v = np.clip(v, 0.0, 1.0)

    pixel_u = (u * (image_width - 1)).astype(np.int32)
    pixel_v = (v * (image_height - 1)).astype(np.int32)
    return pixel_u, pixel_v
