"""Hilfsfunktionen zur Farbgebung auf Basis von Intensitäten."""

from __future__ import annotations

import numpy as np


def intensity_to_rgb(intensities: np.ndarray) -> np.ndarray:
    """Konvertiert Intensitätswerte (0..1) in RGB-Farben.

    Es wird eine einfache blau-grün-rot Farbskala verwendet, die für Einsteiger
    leicht nachvollziehbar ist. Die Funktion arbeitet vollständig mit NumPy und ist
    daher gut testbar.
    """

    intensities = np.clip(intensities, 0.0, 1.0)
    red = intensities
    green = 1.0 - np.abs(intensities - 0.5) * 2.0
    blue = 1.0 - intensities
    colors = np.stack([red, green, blue], axis=-1)
    return (colors * 255).astype(np.uint8)
