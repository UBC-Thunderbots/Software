import os

# Suppress the pygame "Hello from the pygame community" banner on import
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"

from pygame._sdl2 import controller


class HandheldController:
    """Represents a handheld game controller (e.g. an Xbox gamepad) that can be
    used to manually control our robots.

    Backed by SDL's game-controller API (via pygame), which recognizes
    controllers through its controller database and exposes a standardized
    button/axis layout. This means a recognized controller works the same way
    across platforms (Linux, macOS), and callers can read inputs by their
    semantic SDL constant (e.g. pygame.CONTROLLER_AXIS_LEFTX,
    pygame.CONTROLLER_BUTTON_A).
    """

    # Raw SDL axis values span the signed 16-bit range; used to normalize the
    # stick/trigger readings to [-1, 1] / [0, 1].
    _AXIS_MAX_VALUE = 32767

    def __init__(self, controller_index: int):
        """Open the game controller at the given SDL device index.

        :param controller_index: the SDL device index of the controller
        """
        self.controller = controller.Controller(controller_index)

    @classmethod
    def detect(cls) -> "HandheldController | None":
        """Scan the currently connected devices for a supported game controller.

        :return: a HandheldController for the first recognized controller, or
            None if none is connected
        """
        controller.init()
        # Poll mode: refresh state via update() instead of the SDL event queue,
        # so no display / event subsystem is required (Thunderscope drives this
        # widget by polling, and pygame's event pump needs a video system).
        controller.set_eventstate(False)
        controller.update()

        for index in range(controller.get_count()):
            if controller.is_controller(index):
                return cls(index)

        return None

    def name(self) -> str:
        """Get the device name of the controller.

        :return: the device name
        """
        return self.controller.name

    def connected(self) -> bool:
        """Check whether the controller is currently connected.

        :return: true if the controller is connected, false otherwise
        """
        return self.controller.attached()

    def update(self) -> None:
        """Refresh the controller's input state.

        Must be called before reading input values, since we run SDL in poll
        mode rather than consuming its event queue.
        """
        controller.update()

    def key_down(self, button: int) -> bool:
        """Check whether the given button on the controller is currently pressed.

        :param button: the pygame.CONTROLLER_BUTTON_* code of the button to check
        :return: true if the button is currently pressed down, false otherwise
        """
        return bool(self.controller.get_button(button))

    def abs_value(self, axis: int) -> float:
        """Get the current value of an axis input on the controller, normalized
        to a range of [-1, 1] for sticks and [0, 1] for triggers.

        :param axis: the pygame.CONTROLLER_AXIS_* code of the axis to read
        :return: the normalized value of the axis input
        """
        raw_value = self.controller.get_axis(axis)
        return max(-1.0, min(1.0, raw_value / HandheldController._AXIS_MAX_VALUE))

    def close(self) -> None:
        """Close the connection to the controller."""
        self.controller.quit()
