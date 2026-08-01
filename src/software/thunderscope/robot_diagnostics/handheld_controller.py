import os

# Suppress the pygame "Hello from the pygame community" banner on import
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"

import pygame
from pygame._sdl2 import controller

from software.thunderscope.constants import DiagnosticsConstants
from software.thunderscope.robot_diagnostics.controller_base import ControllerBase


class HandheldController(ControllerBase):
    """Represents a handheld game controller (e.g. an Xbox gamepad) that can be
    used to manually control our robots.

    Backed by SDL's game-controller API (via pygame), which recognizes controllers
    through its controller database and exposes a standardized button/axis layout.
    Controllers are read in poll mode — call update() each frame before reading inputs.
    """

    # Raw SDL axis values span the signed 16-bit range.
    _AXIS_MAX_VALUE = 32767

    def __init__(self, controller_index: int):
        """Open the game controller at the given SDL device index.

        :param controller_index: the SDL device index of the controller
        """
        self.controller = controller.Controller(controller_index)

        self._last_dpad_x = 0
        self._last_dpad_y = 0
        self._last_btn_a = False
        self._last_btn_b = False

    @classmethod
    def detect(cls) -> "HandheldController | None":
        """Scan the currently connected devices for a supported game controller.

        :return: a HandheldController for the first recognized controller, or
            None if none is connected
        """
        controller.init()
        # Poll mode: refresh state via update() instead of the SDL event queue,
        # so no display/event subsystem is required.
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
        """Refresh the controller's input state from SDL.

        Must be called once per frame before reading any input values.
        """
        controller.update()

    def close(self) -> None:
        """Close the connection to the controller."""
        self.controller.quit()

    def get_move_velocity(self) -> tuple[float, float, float]:
        """Return (x, y, angular) velocity normalized to [-1, 1] with deadzone applied.

        :return: (vx, vy, vrot) where positive x = forward, positive y = strafe left,
                 positive angular = CCW
        """

        def with_deadzone(v: float) -> float:
            return v if abs(v) >= DiagnosticsConstants.DEADZONE_PERCENTAGE else 0.0

        # Negate: SDL stick-up is negative LEFTY, but semantic "forward" = +vx.
        vx = -with_deadzone(self._get_abs_normalized(pygame.CONTROLLER_AXIS_LEFTY))
        vy = -with_deadzone(self._get_abs_normalized(pygame.CONTROLLER_AXIS_LEFTX))
        vrot = -with_deadzone(self._get_abs_normalized(pygame.CONTROLLER_AXIS_RIGHTX))
        return vx, vy, vrot

    def get_speed_factor(self) -> float:
        """Return 1.0 normally, or SPEED_SLOWDOWN_FACTOR when the left trigger is held.

        :return: the speed scaling factor
        """
        trigger = self._get_abs_normalized(pygame.CONTROLLER_AXIS_TRIGGERLEFT)
        return (
            DiagnosticsConstants.SPEED_SLOWDOWN_FACTOR
            if trigger > DiagnosticsConstants.BUTTON_PRESSED_THRESHOLD
            else 1.0
        )

    def is_dribbler_held(self) -> bool:
        """Return True if the right trigger is held past the button threshold.

        :return: true if the dribbler engage input is active
        """
        return (
            self._get_abs_normalized(pygame.CONTROLLER_AXIS_TRIGGERRIGHT)
            > DiagnosticsConstants.BUTTON_PRESSED_THRESHOLD
        )

    def get_kick_power_step(self) -> int:
        """Return the kick/chip power step direction from the d-pad x buttons.
        Non-zero only on a new press (edge-detected).

        :return: -1, 0, or +1
        """
        current = int(
            self.controller.get_button(pygame.CONTROLLER_BUTTON_DPAD_RIGHT)
        ) - int(self.controller.get_button(pygame.CONTROLLER_BUTTON_DPAD_LEFT))
        step = current if current != self._last_dpad_x else 0
        self._last_dpad_x = current
        return step

    def get_dribbler_step(self) -> int:
        """Return the dribbler RPM step direction from the d-pad y buttons.
        Non-zero only on a new press (edge-detected). Positive = increase RPM.

        :return: -1, 0, or +1
        """
        current = int(
            self.controller.get_button(pygame.CONTROLLER_BUTTON_DPAD_DOWN)
        ) - int(self.controller.get_button(pygame.CONTROLLER_BUTTON_DPAD_UP))
        # Negate: down = +1 raw → decrease dribbler → -1; up = -1 raw → increase → +1.
        step = -current if current != self._last_dpad_y else 0
        self._last_dpad_y = current
        return step

    def is_kick_fired(self) -> bool:
        """Return True once on the rising edge of the kick button (A).

        :return: true if the kick button was just pressed
        """
        current = bool(self.controller.get_button(pygame.CONTROLLER_BUTTON_A))
        fired = current and not self._last_btn_a
        self._last_btn_a = current
        return fired

    def is_chip_fired(self) -> bool:
        """Return True once on the rising edge of the chip button (B).

        :return: true if the chip button was just pressed
        """
        current = bool(self.controller.get_button(pygame.CONTROLLER_BUTTON_B))
        fired = current and not self._last_btn_b
        self._last_btn_b = current
        return fired

    def _get_abs_normalized(self, axis: int) -> float:
        """Return the current value of an axis, normalized to [-1, 1].

        :param axis: the pygame.CONTROLLER_AXIS_* code of the axis to read
        :return: normalized axis value in [-1, 1]
        """
        return max(
            -1.0, min(1.0, self.controller.get_axis(axis) / self._AXIS_MAX_VALUE)
        )
