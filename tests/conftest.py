import sys
import types
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))

fake_gpio = types.ModuleType("RPi.GPIO")
fake_gpio.BCM = "BCM"
fake_gpio.OUT = "OUT"
fake_gpio.HIGH = 1
fake_gpio.LOW = 0


def dummy(*args, **kwargs):
    pass


fake_gpio.setmode = dummy
fake_gpio.setup = dummy
fake_gpio.output = dummy
fake_gpio.cleanup = dummy

sys.modules["RPi.GPIO"] = fake_gpio
