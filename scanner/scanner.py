from math import cos, sin, radians
from typing import List

from .lidar import STL27L
from .pointcloud import Point
from .stepper import Stepper


class Scanner:
    def __init__(
        self, lidar: STL27L, stepper: Stepper, angle_step: float = 1.0
    ) -> None:
        self.lidar = lidar
        self.stepper = stepper
        self.angle_step = angle_step
        self.points: List[Point] = []

    def scan(self, rotations: int = 1) -> List[Point]:
        self.lidar.open()
        angle = 0.0
        steps_per_rotation = int(360 / self.angle_step)
        total_steps = steps_per_rotation * rotations
        for step_count in range(total_steps):
            distance = self.lidar.read_distance()
            rad = radians(angle)
            x = distance * cos(rad)
            y = distance * sin(rad)
            self.points.append(Point(x, y, 0.0))
            self.stepper.step(1)
            angle += self.angle_step
        self.lidar.close()
        return self.points
