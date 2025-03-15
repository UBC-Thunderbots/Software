import os
from enum import IntEnum
import software.python_bindings as tbots_cpp

DEFAULT_PRIMITIVE_DURATION = 2.0
ROBOT_CONSTANTS = tbots_cpp.create2021RobotConstants()

ROBOT_MAX_ANG_SPEED_RAD_PER_S = ROBOT_CONSTANTS.robot_max_ang_speed_rad_per_s
ROBOT_MAX_SPEED_M_PER_S = ROBOT_CONSTANTS.robot_max_speed_m_per_s
MAX_FORCE_DRIBBLER_SPEED_RPM = ROBOT_CONSTANTS.max_force_dribbler_speed_rpm


class EstopMode(IntEnum):
    """Enum for the various estop modes we can run thunderscope in

    DISABLE_ESTOP: No physical / keyboard estop is needed, but we cannot send anything over the network
    KEYBOARD_ESTOP: The spacebar can be used as an estop toggle instead of a physical estop
    PHYSICAL_ESTOP: A physical estop is needed to run thunderscope, throws an exception if none is plugged in
    """

    DISABLE_ESTOP = 0
    KEYBOARD_ESTOP = 1
    PHYSICAL_ESTOP = 2


# Paths to check for estop when running diagnostics
ESTOP_PATH_1 = "/dev/ttyACM0"
ESTOP_PATH_2 = "/dev/EstopUSB"


def get_estop_config(
    keyboard_estop: bool, disable_communication: bool
) -> tuple[EstopMode, os.PathLike]:
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

    # use different estop based on what is plugged in for physical estop mode
    if mode == EstopMode.PHYSICAL_ESTOP:
        path = (
            ESTOP_PATH_1
            if os.path.exists(ESTOP_PATH_1)
            else ESTOP_PATH_2
            if os.path.exists(ESTOP_PATH_2)
            else None
        )
        if not path:
            raise Exception(
                "Estop is not plugged into a valid port, plug one in or use a different estop mode"
            )

    return mode, path
