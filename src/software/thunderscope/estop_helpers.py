import os
from serial.tools import list_ports

from software.py_constants import ARDUINO_VENDOR_ID, ARDUINO_PRODUCT_ID
from software.thunderscope.constants import EstopMode, ESTOP_PATH_1, ESTOP_PATH_2


def get_estop_path() -> str | None:
    """Find the serial port that the physical estop is connected to.

    Falls back to the well-known Linux device paths if the estop cannot be
    matched by its USB IDs.

    :return: the estop's serial port path, or None if no estop was found
    """
    for port in list_ports.comports():
        if port.vid is None or port.pid is None:
            continue
        if (
            f"{port.vid:04x}".lower() == ARDUINO_VENDOR_ID.lower()
            and f"{port.pid:04x}".lower() == ARDUINO_PRODUCT_ID.lower()
        ):
            return port.device

    # Fall back to linux device paths
    for path in (ESTOP_PATH_1, ESTOP_PATH_2):
        if os.path.exists(path):
            return path

    return None


def get_estop_config(
    keyboard_estop: bool, disable_communication: bool
) -> tuple[EstopMode, str | None]:
    """Based on the estop mode argument provided, gets the corresponding
    estop mode and estop path (defined for physical estop mode only)
    Defaults to Physical estop if the given args are both False

    :param keyboard_estop: True if keyboard estop mode is enabled
    :param disable_communication: True if disable communications mode is enableds
    :return: tuple of estop mode enum value and estop path if needed
    """
    mode = EstopMode.PHYSICAL_ESTOP
    path = None

    if keyboard_estop:
        mode = EstopMode.KEYBOARD_ESTOP
    if disable_communication:
        mode = EstopMode.DISABLE_ESTOP

    # locate the physical estop's serial port when in physical estop mode
    if mode == EstopMode.PHYSICAL_ESTOP:
        path = get_estop_path()
        if not path:
            raise Exception(
                "Estop is not plugged into a valid port, plug one in or use a different estop mode"
            )

    return mode, path
