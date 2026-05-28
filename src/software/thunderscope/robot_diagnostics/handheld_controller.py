import numpy

# TODO: remove the try-catch when we rewrite this with macOS-compatible lib
try:
    from evdev import InputDevice, ecodes
except ImportError:
    pass

from threading import Thread


class HandheldController:
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

    def key_down(self, key_code: int) -> bool:
        """Check whether the given key on the controller is currently pressed down.

        :param key_code: the EV_KEY code of the key to check
        :return: true if the key is currently pressed down, false otherwise
        """
        if key_code not in self.supported_keys:
            return False

        return bool(self.input_values.get(key_code, 0))

    def abs_value(self, abs_code: int) -> float:
        """Get the current value of an absolute axis input on the controller,
        normalized to a range of [-1, 1].

        :param abs_code: the EV_ABS code of the axis to check
        :return: the value of the axis input, normalized to [-1, 1]
        """
        if abs_code not in self.abs_info:
            return 0

        value_min = self.abs_info[abs_code].min
        value_max = self.abs_info[abs_code].max
        value = self.input_values.get(abs_code, 0)

        if value >= 0:
            return numpy.interp(value, (0, value_max), (0, 1))
        else:
            return numpy.interp(value, (value_min, 0), (-1, 0))

    def close(self) -> None:
        """Close the connection to the controller."""
        if self.input_device is not None:
            self.input_values.clear()
            self.input_device.close()
            self.input_device = None

    def __read_input(self) -> None:
        """Endless loop that reads input events from the controller."""
        try:
            for event in self.input_device.read_loop():
                if event.type == ecodes.EV_KEY or event.type == ecodes.EV_ABS:
                    self.input_values[event.code] = event.value
        except:
            self.close()
