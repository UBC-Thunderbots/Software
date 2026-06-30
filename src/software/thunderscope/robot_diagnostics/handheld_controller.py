import numpy

# TODO: remove the try-catch when we rewrite this with macOS-compatible lib
try:
    from evdev import InputDevice, ecodes
except ImportError:
    pass

from threading import Thread

from software.thunderscope.constants import DiagnosticsConstants
from software.thunderscope.robot_diagnostics.controller_base import ControllerBase


class HandheldController(ControllerBase):
    """Represents a handheld game controller or input device that can be used
    to manually control our robots.
    """

    def __init__(self, path: str):
        """Initialize a HandheldController that reads input events from
        the specified device path.

        :param path: the device input path
        """
        self.input_device = InputDevice(path)
        self.input_values: dict[int, float] = {}

        capabilities = self.input_device.capabilities()
        self.supported_keys = set(capabilities[ecodes.EV_KEY])
        self.abs_info = dict(capabilities[ecodes.EV_ABS])

        self._last_hat0x = 0
        self._last_hat0y = 0
        self._last_btn_a = False
        self._last_btn_b = False

        self.thread = Thread(target=self.__read_input, daemon=True)
        self.thread.start()

    def name(self) -> str:
        """Get the device name of the controller.

        :return: the device name
        """
        return self.input_device.name

    def connected(self) -> bool:
        """Check whether the controller is currently connected.

        :return: true if the controller is connected, false otherwise
        """
        return self.input_device is not None

    def close(self) -> None:
        """Close the connection to the controller."""
        if self.input_device is not None:
            self.input_values.clear()
            self.input_device.close()
            self.input_device = None

    def get_move_velocity(self) -> tuple[float, float, float]:
        """Return (x, y, angular) velocity normalized to [-1, 1] with deadzone applied.

        :return: (vx, vy, vrot) where positive x = forward, positive y = strafe left,
                 positive angular = CCW
        """

        def with_deadzone(v: float) -> float:
            return v if abs(v) >= DiagnosticsConstants.DEADZONE_PERCENTAGE else 0.0

        # Negate raw axis values: on most controllers, stick-up is negative ABS_Y,
        # but semantic "forward" should be positive.
        vx = -with_deadzone(self._get_abs_normalized(ecodes.ABS_Y))
        vy = -with_deadzone(self._get_abs_normalized(ecodes.ABS_X))
        vrot = -with_deadzone(self._get_abs_normalized(ecodes.ABS_RX))
        return vx, vy, vrot

    def get_speed_factor(self) -> float:
        """Return 1.0 normally, or SPEED_SLOWDOWN_FACTOR when the left trigger is held.

        :return: the speed scaling factor
        """
        trigger = self._get_abs_normalized(ecodes.ABS_Z)
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
            self._get_abs_normalized(ecodes.ABS_RZ)
            > DiagnosticsConstants.BUTTON_PRESSED_THRESHOLD
        )

    def get_kick_power_step(self) -> int:
        """Return the kick/chip power step direction from the d-pad x axis.
        Non-zero only on a new press (edge-detected).

        :return: -1, 0, or +1
        """
        current = round(self._get_abs_normalized(ecodes.ABS_HAT0X))
        step = current if current != self._last_hat0x else 0
        self._last_hat0x = current
        return step

    def get_dribbler_step(self) -> int:
        """Return the dribbler RPM step direction from the d-pad y axis.
        Non-zero only on a new press (edge-detected). Positive = increase RPM.

        :return: -1, 0, or +1
        """
        current = round(self._get_abs_normalized(ecodes.ABS_HAT0Y))
        # Negate: d-pad up is HAT0Y=-1, but "increase dribbler" should be +1.
        step = -current if current != self._last_hat0y else 0
        self._last_hat0y = current
        return step

    def is_kick_fired(self) -> bool:
        """Return True once on the rising edge of the kick button (BTN_A).

        :return: true if the kick button was just pressed
        """
        current = bool(self.input_values.get(ecodes.BTN_A, 0))
        fired = current and not self._last_btn_a
        self._last_btn_a = current
        return fired

    def is_chip_fired(self) -> bool:
        """Return True once on the rising edge of the chip button (BTN_B).

        :return: true if the chip button was just pressed
        """
        current = bool(self.input_values.get(ecodes.BTN_B, 0))
        fired = current and not self._last_btn_b
        self._last_btn_b = current
        return fired

    def _get_abs_normalized(self, abs_code: int) -> float:
        """Return the current value of an absolute axis, normalized to [-1, 1].

        :param abs_code: the EV_ABS code of the axis to read
        :return: normalized axis value in [-1, 1]
        """
        if abs_code not in self.abs_info:
            return 0.0

        value_min = self.abs_info[abs_code].min
        value_max = self.abs_info[abs_code].max
        value = self.input_values.get(abs_code, 0)

        if value >= 0:
            return numpy.interp(value, (0, value_max), (0, 1))
        else:
            return numpy.interp(value, (value_min, 0), (-1, 0))

    def __read_input(self) -> None:
        """Endless loop that reads input events from the controller."""
        try:
            for event in self.input_device.read_loop():
                if event.type == ecodes.EV_KEY or event.type == ecodes.EV_ABS:
                    self.input_values[event.code] = event.value
        except:
            self.close()
