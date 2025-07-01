import serial
from typing import Optional


class STL27L:
    """Simple driver for the STL27L lidar connected via USB."""

    def __init__(
        self, port: str = "/dev/ttyUSB0", baudrate: int = 115200
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None

    def open(self) -> None:
        """Open the serial connection to the lidar."""
        self.ser = serial.Serial(self.port, self.baudrate, timeout=1)

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    def read_distance(self) -> float:
        """Read a single distance measurement in meters."""
        if not self.ser:
            raise RuntimeError("Serial port not open")
        line = self.ser.readline().decode(errors="ignore").strip()
        try:
            return float(line)
        except ValueError as exc:
            raise RuntimeError(f"Invalid data: {line}") from exc
