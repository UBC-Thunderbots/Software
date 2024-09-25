import time
from logging import Logger
from threading import Thread, RLock

import numpy
from evdev import InputDevice, ecodes, list_devices, InputEvent
from pyqtgraph.Qt import QtCore

import software.python_bindings as tbots_cpp
from proto.import_all_protos import *
from software.thunderscope.constants import *
from software.thunderscope.constants import DiagnosticsConstants
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.robot_diagnostics.handheld_device_status_view import (
    HandheldDeviceConnectionStatus,
)

from dataclasses import dataclass


@dataclass
class HandheldDeviceConnectionChangedEvent:
    connection_status: HandheldDeviceConnectionStatus
    """The new handheld device connection status"""

    device_name: str | None
    """The name of the connected handheld device"""


class HandheldDeviceManager(QtCore.QObject):
    """This class is responsible for managing the connection between a computer and a
    handheld device to control robots. This class relies on the `evdev` python package
    in order to implement parsing and control flow for device events.
    See: https://python-evdev.readthedocs.io/en/latest/apidoc.html
    """

    handheld_device_connection_status_signal = QtCore.pyqtSignal(
        HandheldDeviceConnectionChangedEvent
    )
    """Signal emitted when the handheld device connection status changes"""

    def __init__(
        self,
        logger: Logger,
        proto_unix_io: ProtoUnixIO,
    ):
        """Initialize the HandheldDeviceManager

        :param logger: the logger to use
        :param proto_unix_io: ProtoUnixIO to send messages to the robot
        """
        super().__init__()

        self.lock = RLock()

        self.logger = logger
        self.proto_unix_io = proto_unix_io

        self.enabled = False

        self.handheld_device: Optional[InputDevice] = None
        self.handheld_device_config: Optional[DeviceConfig] = None

        self.__initialize_default_motor_control_values()
        self.__initialize_default_power_control_values()

        # The following fields are here to temporarily persist device input.
        # They are read once certain buttons are pressed on the handheld device,
        # and the values are set for the control primitives above.
        self.kick_power: float = DiagnosticsConstants.MIN_KICK_POWER
        self.chip_distance: float = DiagnosticsConstants.MIN_CHIP_POWER
        self.dribbler_speed: int = 0
        self.dribbler_running: bool = False

        self.constants = tbots_cpp.create2021RobotConstants()

        self.event_loop_thread = Thread(target=self.__event_loop, daemon=True)
        self.event_loop_thread.start()

    def __initialize_default_motor_control_values(self) -> None:
        """Set all required fields in the motor control proto to their
        default/minimum values
        """
        self.motor_control = MotorControl()
        self.motor_control.direct_velocity_control.velocity.x_component_meters = 0.0
        self.motor_control.direct_velocity_control.velocity.y_component_meters = 0.0
        self.motor_control.direct_velocity_control.angular_velocity.radians_per_second = 0.0
        self.motor_control.dribbler_speed_rpm = 0

    def __initialize_default_power_control_values(self) -> None:
        """Set all required fields in the power control proto to their
        default/minimum values
        """
        self.power_control = PowerControl()
        self.power_control.geneva_slot = 3

    def set_enabled(self, enabled) -> None:
        """Enable or disable the HandheldDeviceManager from processing input events.

        :param enabled: whether to enable (True) or disable (False) the
                        HandheldDeviceManager from processing input events
        """
        with self.lock:
            self.enabled = enabled

    def reinitialize_handheld_device(self) -> None:
        """Attempt to reinitialize a connection to a handheld device."""
        self.__clear_handheld_device()
        self.__initialize_handheld_device()

    def __initialize_handheld_device(self) -> None:
        """Attempt to initialize a connection to a handheld device.
        The first handheld device that is recognized as valid will be used.
        Valid handheld devices are any devices whose name has a matching DeviceConfig
        in DiagnosticsConstants.HANDHELD_DEVICE_NAME_CONFIG_MAP.
        """
        device_config_map = DiagnosticsConstants.HANDHELD_DEVICE_NAME_CONFIG_MAP

        for device in list_devices():
            handheld_device = InputDevice(device)
            if (
                handheld_device is not None
                and handheld_device.name in device_config_map
            ):
                with self.lock:
                    self.handheld_device = handheld_device
                    self.handheld_device_config = device_config_map[
                        handheld_device.name
                    ]

                    self.handheld_device_connection_status_signal.emit(
                        HandheldDeviceConnectionChangedEvent(
                            HandheldDeviceConnectionStatus.CONNECTED,
                            self.handheld_device.name,
                        )
                    )

                    self.logger.debug("Successfully initialized handheld!")
                    self.logger.debug(
                        'Device name: "' + self.handheld_device.name + '"'
                    )
                    self.logger.debug("Device path: " + self.handheld_device.path)

                return

        self.__clear_handheld_device()
        self.logger.debug("Failed to initialize handheld device, check USB connection")
        self.logger.debug("Tried the following available devices:")
        self.logger.debug(list(map(lambda d: InputDevice(d).name, list_devices())))

    def __clear_handheld_device(self) -> None:
        """Clear the handheld device and config fields by setting them to None,
        and emit a disconnected notification signal.
        """
        with self.lock:
            self.handheld_device = None
            self.handheld_device_config = None

        self.handheld_device_connection_status_signal.emit(
            HandheldDeviceConnectionChangedEvent(
                HandheldDeviceConnectionStatus.DISCONNECTED, None
            )
        )

    def __event_loop(self) -> None:
        """Loop that continuously reads and processes events from the connected handheld device."""
        while True:
            with self.lock:
                self.__read_and_process_event()
            time.sleep(DiagnosticsConstants.EVENT_LOOP_SLEEP_DURATION)

    def __read_and_process_event(self) -> None:
        """Try reading an event from the connected handheld device and process it."""
        if not self.handheld_device:
            return

        try:
            event = self.handheld_device.read_one()

            # All events accumulate into a file and will be read back eventually,
            # even if handheld mode is disabled. This time based recency check ensures that
            # only the events that have occurred very recently are processed, and
            # any events that occur before switching to handheld mode are dropped & ignored
            if self.enabled and event:
                time_since_event = time.time() - event.timestamp()
                if time_since_event < DiagnosticsConstants.INPUT_DELAY_THRESHOLD:
                    self.__process_event(event)

                    motor_control = MotorControl()
                    power_control = PowerControl()
                    motor_control.CopyFrom(self.motor_control)
                    power_control.CopyFrom(self.power_control)
                    self.proto_unix_io.send_proto(MotorControl, motor_control)
                    self.proto_unix_io.send_proto(PowerControl, power_control)

                    self.__initialize_default_power_control_values()

        except OSError as ose:
            self.__clear_handheld_device()
            self.logger.debug(
                "Caught an OSError while reading handheld handheld device event loop!"
            )
            self.logger.debug("Error message: " + str(ose))
            self.logger.debug("Check physical handheld handheld device USB connection!")

        except Exception as e:
            self.__clear_handheld_device()
            self.logger.critical(
                "Caught an unexpected error while reading handheld handheld device event loop!"
            )
            self.logger.critical("Error message: " + str(e))

    def __process_event(self, event: InputEvent) -> None:
        """Process the given device event. Set the corresponding motor and power control values
        based on the event type, using the current config set in self.handheld_device_config

        :param event: The input event to process
        """
        config = self.handheld_device_config

        if event.type == ecodes.EV_ABS:
            if event.code == config.move_x.event_code:
                velocity = self.motor_control.direct_velocity_control.velocity
                velocity.x_component_meters = self.__interpret_move_event_value(
                    event.value,
                    config.move_x.max_value,
                    self.constants.robot_max_speed_m_per_s,
                )

            elif event.code == config.move_y.event_code:
                velocity = self.motor_control.direct_velocity_control.velocity
                velocity.y_component_meters = self.__interpret_move_event_value(
                    -event.value,
                    config.move_y.max_value,
                    self.constants.robot_max_speed_m_per_s,
                )

            elif event.code == config.move_rot.event_code:
                angular_velocity = (
                    self.motor_control.direct_velocity_control.angular_velocity
                )
                angular_velocity.radians_per_second = self.__interpret_move_event_value(
                    -event.value,
                    config.move_rot.max_value,
                    self.constants.robot_max_ang_speed_rad_per_s,
                )

            elif event.code == config.chicker_power.event_code:
                self.kick_power = self.__interpret_kick_event_value(event.value)
                self.chip_distance = self.__interpret_chip_event_value(event.value)

            elif event.code == config.dribbler_speed.event_code:
                self.dribbler_speed = self.__interpret_dribbler_speed_event_value(
                    event.value
                )

            elif event.code == config.dribbler_enable.event_code:
                self.dribbler_running = self.__interpret_dribbler_enabled_event_value(
                    event.value,
                    config.dribbler_enable.max_value,
                )

        elif event.type == ecodes.EV_KEY:
            if event.code == config.kick.event_code and event.value == 1:
                self.power_control.chicker.kick_speed_m_per_s = self.kick_power

            elif event.code == config.chip.event_code and event.value == 1:
                self.power_control.chicker.chip_distance_meters = self.chip_distance

        self.motor_control.dribbler_speed_rpm = (
            self.dribbler_speed if self.dribbler_running else 0
        )

    def __interpret_move_event_value(
        self, event_value: float, max_value: float, normalizing_multiplier: float
    ) -> float:
        """Interpret the event_value that corresponds to movement control

        :param event_value: the value for the current event being interpreted
        :param max_value: max value for this type event type
        :param normalizing_multiplier: multiplier for converting between the event value and the
                                       robot movement control value
        :return: The interpreted value that can be set a value for a field in robot movement control
        """
        relative_value = event_value / max_value
        if abs(relative_value) < DiagnosticsConstants.DEADZONE_PERCENTAGE:
            return 0
        else:
            return relative_value * normalizing_multiplier

    def __interpret_dribbler_enabled_event_value(
        self, event_value: float, max_value: float
    ) -> bool:
        """Interpret the event_value that corresponds to controlling whether the dribbler is enabled

        :param event_value: the value for the current event being interpreted
        :param max_value: max value for this event type
        :return: The interpreted value that can be set a value for a field in robot control
        """
        button_percent_pressed = event_value / max_value
        return button_percent_pressed > DiagnosticsConstants.BUTTON_PRESSED_THRESHOLD

    def __interpret_dribbler_speed_event_value(self, event_value: float) -> int:
        """Interpret the event value that corresponds to controlling the dribbler speed.

        :param event_value: the value for the current event being interpreted
        :return: the interpreted value to be used for the new dribbler speed on the robot
        """
        return int(
            numpy.clip(
                a=self.dribbler_speed
                - event_value * DiagnosticsConstants.DRIBBLER_RPM_STEPPER,
                a_min=self.constants.indefinite_dribbler_speed_rpm,
                a_max=-self.constants.indefinite_dribbler_speed_rpm,
            )
        )

    def __interpret_kick_event_value(self, event_value: float) -> float:
        """Interpret the event value that corresponds to controlling the dribbler speed.

        :param event_value: the value for the current event being interpreted
        :return: the interpreted value to be used for the kick power on the robot
        """
        return numpy.clip(
            a=self.kick_power + event_value * DiagnosticsConstants.KICK_POWER_STEPPER,
            a_min=DiagnosticsConstants.MIN_KICK_POWER,
            a_max=DiagnosticsConstants.MAX_KICK_POWER,
        )

    def __interpret_chip_event_value(self, event_value: float) -> float:
        """Interpret the event value that corresponds to controlling the chip distance.

        :param value: the value for the current event being interpreted
        :return: the interpreted value to be used for the chip distance on the robot
        """
        return numpy.clip(
            a=self.chip_distance
            + event_value * DiagnosticsConstants.CHIP_DISTANCE_STEPPER,
            a_min=DiagnosticsConstants.MIN_CHIP_POWER,
            a_max=DiagnosticsConstants.MAX_CHIP_POWER,
        )
