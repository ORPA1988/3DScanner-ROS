"""Abstrakte Ansteuerung des NEMA17-Schrittmotors über den A4988-Treiber."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

try:
    import RPi.GPIO as GPIO
except ImportError:  # pragma: no cover - wird im CI ohne Hardware nicht ausgeführt
    GPIO = None  # type: ignore


@dataclass
class StepperConfig:
    """Konfigurationsdatensatz für den Schrittmotor."""

    step_pin: int
    dir_pin: int
    enable_pin: int
    steps_per_revolution: float
    max_speed: float


class StepperDriver:
    """Kapselt grundlegende Bewegungsbefehle für den A4988-Treiber."""

    def __init__(self, config: StepperConfig) -> None:
        if GPIO is None:
            raise RuntimeError(
                "RPi.GPIO konnte nicht importiert werden. Stellen Sie sicher, dass das Skript auf einem Raspberry Pi läuft."
            )

        self._config = config
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(config.step_pin, GPIO.OUT)
        GPIO.setup(config.dir_pin, GPIO.OUT)
        GPIO.setup(config.enable_pin, GPIO.OUT)
        GPIO.output(config.enable_pin, GPIO.LOW)

        self._current_angle = 0.0

    @property
    def current_angle(self) -> float:
        return self._current_angle

    def rotate_to(self, target_angle: float, angular_velocity: float) -> None:
        angular_velocity = min(abs(angular_velocity), self._config.max_speed)
        direction = GPIO.HIGH if target_angle > self._current_angle else GPIO.LOW
        GPIO.output(self._config.dir_pin, direction)

        angle_difference = abs(target_angle - self._current_angle)
        steps_needed = int((angle_difference / (2 * math.pi)) * self._config.steps_per_revolution)
        delay = max(1.0 / (angular_velocity * self._config.steps_per_revolution / (2 * math.pi)), 0.0005)

        for _ in range(steps_needed):
            GPIO.output(self._config.step_pin, GPIO.HIGH)
            time.sleep(delay / 2)
            GPIO.output(self._config.step_pin, GPIO.LOW)
            time.sleep(delay / 2)

        self._current_angle = target_angle

    def disable(self) -> None:
        GPIO.output(self._config.enable_pin, GPIO.HIGH)

    def enable(self) -> None:
        GPIO.output(self._config.enable_pin, GPIO.LOW)

    def cleanup(self) -> None:
        GPIO.cleanup()
