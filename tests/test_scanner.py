from scanner.scanner import Scanner
from scanner.pointcloud import Point


class FakeLidar:
    def __init__(self, distances):
        self.distances = distances
        self.index = 0

    def open(self):
        pass

    def close(self):
        pass

    def read_distance(self):
        d = self.distances[self.index]
        self.index += 1
        return d


class FakeStepper:
    def __init__(self):
        self.steps = []

    def step(self, steps, direction=1, delay=0.001):
        self.steps.append(steps)

    def cleanup(self):
        pass


def test_scanner_scan():
    lidar = FakeLidar([1.0, 1.0])
    stepper = FakeStepper()
    scanner = Scanner(lidar, stepper, angle_step=180)
    points = scanner.scan(rotations=1)
    assert len(points) == 2
    assert isinstance(points[0], Point)
    assert stepper.steps == [1, 1]
