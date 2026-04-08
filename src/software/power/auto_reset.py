Import("env")
import time
import os
import subprocess
from enum import Enum

BOOT_GPIO = 24
RESET_GPIO = 23

"""
This script is deployed onto the remote device and configures automatic reset/boot for the target esp32 to be flashed.
"""


class PinState(Enum):
    LOW = 0
    HIGH = 1


def sysfs_gpio(pin, value):
    if not os.path.exists(f"/sys/class/gpio/gpio{pin}"):
        with open("/sys/class/gpio/export", "w") as f:
            f.write(str(pin))
        time.sleep(0.1)
    with open(f"/sys/class/gpio/gpio{pin}/direction", "w") as f:
        f.write("out")
    with open(f"/sys/class/gpio/gpio{pin}/value", "w") as f:
        f.write(str(value.value))


def set_gpio(pin, value):
    try:
        sysfs_gpio(pin, value)
    except Exception:
        v = "dh" if value == PinState.HIGH else "dl"
        try:
            subprocess.run(
                ["pinctrl", "set", str(pin), "op", v],
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception:
            subprocess.run(
                ["raspi-gpio", "set", str(pin), "op", v],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )


def before_upload(source, target, env):
    print(
        f"Setting ESP32 to bootloader mode using GPIO {BOOT_GPIO} (BOOT) and GPIO {RESET_GPIO} (EN)..."
    )
    set_gpio(BOOT_GPIO, PinState.LOW)
    time.sleep(0.1)
    set_gpio(RESET_GPIO, PinState.LOW)
    time.sleep(0.1)
    set_gpio(RESET_GPIO, PinState.HIGH)
    time.sleep(0.5)
    set_gpio(BOOT_GPIO, PinState.HIGH)


def after_upload(source, target, env):
    print("Resetting ESP32...")
    set_gpio(RESET_GPIO, PinState.LOW)
    time.sleep(0.1)
    set_gpio(RESET_GPIO, PinState.HIGH)


env.AddPreAction("upload", before_upload)
env.AddPostAction("upload", after_upload)
