from time import sleep

try:
    import RPi.GPIO as GPIO  # type: ignore
except Exception:  # pragma: no cover - fallback for non-Pi environments
    class GPIO:  # type: ignore
        """Minimal stand-in for RPi.GPIO when running off-device."""

        BCM = "BCM"
        OUT = "OUT"
        HIGH = 1
        LOW = 0

        @staticmethod
        def setmode(mode):
            pass

        @staticmethod
        def setup(pin, mode):
            pass

        @staticmethod
        def output(pin, state):
            pass

        @staticmethod
        def cleanup():
            pass


class Stepper:
    """Control a NEMA17 stepper motor via A4988 driver."""

    def __init__(
        self, step_pin: int, dir_pin: int, enable_pin: int | None = None
    ) -> None:
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.enable_pin = enable_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin, GPIO.OUT)
        if self.enable_pin is not None:
            GPIO.setup(self.enable_pin, GPIO.OUT)
            GPIO.output(self.enable_pin, GPIO.LOW)

    def step(
        self, steps: int, direction: int = 1, delay: float = 0.001
    ) -> None:
        GPIO.output(self.dir_pin, GPIO.HIGH if direction > 0 else GPIO.LOW)
        for _ in range(steps):
            GPIO.output(self.step_pin, GPIO.HIGH)
            sleep(delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            sleep(delay)

    def cleanup(self) -> None:
        GPIO.cleanup()
