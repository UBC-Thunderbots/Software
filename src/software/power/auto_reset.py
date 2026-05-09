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

def sysfs_gpio(pin, value) -> None:
    """
    Configures the pin and sets the value of the GPIO pin using the file system
    :param pin: Target pin
    :param value: Value to set to
    """
    if not os.path.exists(f"/sys/class/gpio/gpio{pin}"):
        with open("/sys/class/gpio/export", "w") as f:
            f.write(str(pin))
        time.sleep(0.1)
    with open(f"/sys/class/gpio/gpio{pin}/direction", "w") as f:
        f.write("out")
    with open(f"/sys/class/gpio/gpio{pin}/value", "w") as f:
        f.write(str(value.value))

def set_gpio(pin, value) -> None:
    """
    Sets the value of the given GPIO pin. Uses High or Low only on fallback.
    :param pin: Pin to set
    :param value: Value to set to
    """
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


def before_upload(source, target, env) -> None:
    """
    Action to be run before firmware flashing.

    :param source: Compiled firmware
    :param target: Build action name
    :param env: Environment variables
    """
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


def after_upload(source, target, env) -> None:
    """
    Action to be run after firmware flashing.

    :param source: Compiled firmware
    :param target: Build action name
    :param env: Environment variables
    """
    print("Resetting ESP32...")
    set_gpio(RESET_GPIO, PinState.LOW)
    time.sleep(0.1)
    set_gpio(RESET_GPIO, PinState.HIGH)

# Attach pre-upload and post-upload hooks
env.AddPreAction("upload", before_upload)
env.AddPostAction("upload", after_upload)
