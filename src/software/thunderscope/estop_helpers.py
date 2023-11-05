import os
from typing import Tuple, Optional
from software.thunderscope.constants import EstopMode, ESTOP_PATH_1, ESTOP_PATH_2
from argparse import Namespace


def get_estop_config(args: Namespace) -> Tuple[EstopMode, Optional[os.PathLike]]:
    """
    Based on the estop mode argument provided, gets the corresponding
    estop mode and estop path (defined for physical estop mode only)
    :return: tuple of estop mode enum value and estop path if needed
    """

    mode = EstopMode.PHYSICAL_ESTOP
    path = None

    if args.keyboard_estop:
        mode = EstopMode.KEYBOARD_ESTOP
    if args.disable_communication:
        mode = EstopMode.DISABLE_ESTOP

    # use different estop based on what is plugged in for physical estop mode
    if mode == EstopMode.PHYSICAL_ESTOP:
        path = (
            ESTOP_PATH_1
            if os.path.isfile(ESTOP_PATH_1)
            else ESTOP_PATH_2
            if os.path.isfile(ESTOP_PATH_2)
            else None
        )
        if not path:
            raise Exception(
                "Estop is not plugged into a valid port, plug one in or use a different estop mode"
            )

    return mode, path
