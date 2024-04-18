import time
from logging import Logger
from threading import Thread, Event
from typing import Type

import evdev.ecodes
import numpy
from evdev import InputDevice, ecodes, list_devices, InputEvent

import software.python_bindings as tbots_cpp
from proto.import_all_protos import *
from software.thunderscope.constants import *
from software.thunderscope.constants import HandheldDeviceConstants
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.robot_diagnostics.handheld_device_status_view import (
    HandheldDeviceConnectionStatus,
)
from software.thunderscope.robot_diagnostics.diagnostics_input_widget import ControlMode


# TODO: function docs lollll
# TODO bug with threading & switching back and forth between control types,
#  thread refuses to terminate as usual


class HandheldDeviceManager(object):
    """
    This class is responsible for reading from a handheld controller device and
    interpreting the device inputs into usable inputs for the robots.
    """

    def __init__(
            self,
            proto_unix_io: ProtoUnixIO,
            logger: Logger,
            handheld_device_disconnected_signal: Type[QtCore.pyqtSignal]
    ):
        self.proto_unix_io = proto_unix_io
        self.logger = logger
        self.handheld_device_disconnected_signal = handheld_device_disconnected_signal

        self.controller: Optional[InputDevice] = None
        self.controller_config: Optional[HDIEConfig] = None

        # Control primitives that are directly updated with
        # parsed controller inputs on every iteration of the event loop
        self.motor_control = MotorControl()
        self.power_control = PowerControl()

        # These fields are here to temporarily persist the controller's input.
        # They are read once certain buttons are pressed on the controller,
        # and the values are inserted into the control primitives above.
        self.kick_power_accumulator: float = 0.0
        self.chip_distance_accumulator: float = 0.0
        self.dribbler_speed_accumulator: int = 0

        self.constants = tbots_cpp.create2021RobotConstants()

        # event that is used to stop the controller event loop
        self.__stop_thread_signal_event = Event()

        # Set-up process that will run the event loop
        self.__setup_new_event_listener_thread()

        self.__initialize_controller()

    def __initialize_default_controls(self):
        # default values for motor control
        self.motor_control.direct_velocity_control.velocity.x_component_meters = 0.0
        self.motor_control.direct_velocity_control.velocity.y_component_meters = 0.0
        self.motor_control.direct_velocity_control.angular_velocity.radians_per_second = 0.0
        self.motor_control.dribbler_speed_rpm = 0

        # default values for power control
        self.power_control.geneva_slot = 3
        self.power_control.chicker = ChickerControl()

    def reinitialize_controller(self):
        self.__clear_controller()
        self.__initialize_controller()

    def __initialize_controller(self):
        """
        Attempt to initialize a controller.
        The first controller that is recognized as a valid controller will be used
        """
        for device in list_devices():
            controller = InputDevice(device)
            if (
                    controller is not None
                    and controller.name
                    in HandheldDeviceConstants.CONTROLLER_NAME_CONFIG_MAP
            ):
                self.__stop_thread_signal_event.clear()
                self.controller = controller
                self.controller_config: HDIEConfig = HandheldDeviceConstants.CONTROLLER_NAME_CONFIG_MAP[
                    controller.name
                ]
                break

        if (
                self.__get_handheld_device_connection_status()
                == HandheldDeviceConnectionStatus.CONNECTED
        ):
            self.handheld_device_disconnected_signal.emit(HandheldDeviceConnectionStatus.CONNECTED)
            self.logger.debug('Successfully initialized handheld!')
            self.logger.debug('Device name: \"' + self.controller.name + '\"')
            self.logger.debug('Device path: ' + self.controller.path)

        elif (
                self.__get_handheld_device_connection_status()
                == HandheldDeviceConnectionStatus.DISCONNECTED
        ):
            self.handheld_device_disconnected_signal.emit(HandheldDeviceConnectionStatus.DISCONNECTED)
            self.logger.debug(
                "Failed to initialize a handheld controller, check USB connection"
            )
            self.logger.debug("Tried the following available devices:")
            self.logger.debug(list(map(lambda d: InputDevice(d).name, list_devices())))

    def __get_handheld_device_connection_status(self) -> HandheldDeviceConnectionStatus:
        return (
            HandheldDeviceConnectionStatus.CONNECTED
            if self.controller is not None
            else HandheldDeviceConnectionStatus.DISCONNECTED
        )

    def get_latest_primitive_controls(self):
        diagnostics_primitive = Primitive(
            direct_control=DirectControlPrimitive(
                motor_control=self.motor_control,
                power_control=self.power_control
            )
        )
        # TODO: pre-emptive bugfix: need to reset controls, especially power so that
        #  the control message isn't set to what is essentially auto-kick/chip
        self.power_control.chicker.chip_distance_meters = 0.0
        self.power_control.chicker.kick_speed_m_per_s = 0.0

        return diagnostics_primitive

    def refresh(self, mode: ControlMode):
        """
        Refresh this class.
        Spawns a new thread that runs the handheld device event
        processing function if Control.Mode is Diagnostics,
        otherwise, stops and joins the thread, if it is running
        :param mode: The current user requested mode for controlling the robot
        """
        if mode == ControlMode.DIAGNOSTICS:
            if self.__controller_event_loop_handler_thread.is_alive():
                self.logger.debug("Terminating controller event handling process")
                self.__shutdown_handheld_device_event_loop_handler_thread()
        elif mode == ControlMode.HANDHELD:
            # if not self.__controller_event_loop_handler_thread.is_alive():
            self.logger.debug("Setting up new controller event handling process")
            self.__setup_new_event_listener_thread()
            self.__start_event_listener_thread()

    def __clear_controller(self):
        self.logger.debug(
            f"shutting down handler thread, is_alive = {self.__controller_event_loop_handler_thread.is_alive()}")
        self.__shutdown_handheld_device_event_loop_handler_thread()
        self.logger.debug(
            f"after shutdown handler thread, is_alive = {self.__controller_event_loop_handler_thread.is_alive()}")
        self.controller = None
        self.controller_config = None
        self.handheld_device_disconnected_signal.emit(HandheldDeviceConnectionStatus.DISCONNECTED)

    def close(self):
        self.__shutdown_handheld_device_event_loop_handler_thread()

    def __shutdown_handheld_device_event_loop_handler_thread(self):
        """
        Shut down the controller event loop
        :return:
        """
        # TODO (#3165): Use trace level logging here
        # self.logger.debug("Shutdown down controller event loop process")
        self.__stop_thread_signal_event.set()
        if self.__controller_event_loop_handler_thread.is_alive():
            self.__controller_event_loop_handler_thread.join()
            self.__setup_new_event_listener_thread()

    def __start_event_listener_thread(self):
        # start the process for reading controller events
        self.__controller_event_loop_handler_thread.start()
        self.logger.debug(self.__controller_event_loop_handler_thread.is_alive())

    def __setup_new_event_listener_thread(self):
        """
        initializes a new process that will run the event processing loop
        """

        # TODO (#3165): Use trace level self.logger here
        # self.logger.debug("Starting controller event loop process")
        self.__controller_event_loop_handler_thread = Thread(
            target=self.__event_loop, daemon=True
        )

    def __event_loop(self):
        # TODO (#3165): Use trace level self.logger here
        self.logger.debug("Starting handheld controller event handling loop")
        try:
            while True:
                if self.__stop_thread_signal_event.is_set():
                    return
                event = self.controller.read_one()
                # All events accumulate into a file and will be read back eventually,
                # even if handheld mode is disabled. This time based recency check ensures that
                # only the events that have occurred very recently are processed, and
                # any events that occur before switching to handheld mode are dropped & ignored
                if (
                        event is not None
                        and time.time() - event.timestamp()
                        < HandheldDeviceConstants.INPUT_DELAY_THRESHOLD
                ):
                    self.__process_event(event)

                time.sleep(0.0005)

        except OSError as ose:
            self.__clear_controller()
            self.logger.debug(
                "Caught an OSError while reading handheld controller event loop!"
            )
            self.logger.debug("Error message: " + str(ose))
            self.logger.debug("Check physical handheld controller USB connection!")
            return
        except Exception as e:
            self.__clear_controller()
            self.logger.critical(
                "Caught an unexpected error while reading handheld controller event loop!"
            )
            self.logger.critical("Error message: " + str(e))
            return

    def __process_event(self, event: InputEvent):

        # TODO (#3165): Use trace level self.logger here
        # self.logger.debug(
        #     "Processing controller event with type: "
        #     + str(ecodes.bytype[event.type][event.code])
        #     + ", with code: "
        #     + str(event.code)
        #     + ", and with value: "
        #     + str(event.value)
        # )

        if event.type == ecodes.EV_ABS:
            if event.code == self.controller_config.move_x.event_code:
                self.motor_control.direct_velocity_control.velocity.x_component_meters = self.__parse_move_event_value(
                    event_value=event.value,
                    max_value=self.controller_config.move_x.max_value,
                    normalizing_multiplier=self.constants.robot_max_speed_m_per_s,
                )

            if event.code == self.controller_config.move_y.event_code:
                self.motor_control.direct_velocity_control.velocity.y_component_meters = self.__parse_move_event_value(
                    event_value=event.value,
                    max_value=self.controller_config.move_y.max_value,
                    normalizing_multiplier=self.constants.robot_max_speed_m_per_s,
                )

            if event.code == self.controller_config.move_rot.event_code:
                self.motor_control.direct_velocity_control.angular_velocity.radians_per_second = self.__parse_move_event_value(
                    event_value=event.value,
                    max_value=self.controller_config.move_rot.max_value,
                    normalizing_multiplier=self.constants.robot_max_ang_speed_rad_per_s,
                )

            elif event.code == self.controller_config.chicker_power.event_code:
                self.kick_power_accumulator = self.__parse_kick_event_value(event.value)
                self.chip_distance_accumulator = self.__parse_chip_event_value(event.value)

            elif event.code == self.controller_config.dribbler_speed.event_code:
                self.dribbler_speed_accumulator = self.__parse_dribbler_speed_event_value(
                    event.value
                )

            elif (
                    event.code == self.controller_config.primary_dribbler_enable.event_code
                    or event.code
                    == self.controller_config.secondary_dribbler_enable.event_code
            ):
                dribbler_enabled = self.__parse_dribbler_enabled_event_value(
                    event_value=event.value,
                    max_value=self.controller_config.primary_dribbler_enable.max_value,
                )
                if dribbler_enabled:
                    self.motor_control.dribbler_speed_rpm = (
                        self.dribbler_speed_accumulator
                    )

        if event.type == ecodes.EV_KEY:
            if (
                    event.code == self.controller_config.kick.event_code
                    and event.value == 1
            ):
                self.power_control.geneva_slot = 3
                self.power_control.chicker.kick_speed_m_per_s = (
                    self.kick_power_accumulator
                )

            if (
                    event.code == self.controller_config.chip.event_code
                    and event.value == 1
            ):
                self.power_control.geneva_slot = 3
                self.power_control.chicker.chip_distance_meters = (
                    self.kick_power_accumulator
                )

    @staticmethod
    def __parse_move_event_value(
            event_value: float, max_value: float, normalizing_multiplier: float
    ) -> float:
        relative_value = event_value / max_value
        if abs(relative_value) < HandheldDeviceConstants.DEADZONE_PERCENTAGE:
            return 0
        else:
            return relative_value * normalizing_multiplier

    def __parse_dribbler_enabled_event_value(
            self, event_value: float, max_value: float
    ) -> bool:
        return (event_value / max_value) > 0.5

    def __parse_dribbler_speed_event_value(self, value: float) -> float:
        return numpy.clip(
            a=self.motor_control.dribbler_speed_rpm + value * HandheldDeviceConstants.DRIBBLER_SPEED_STEPPER,
            a_min=0,
            a_max=HandheldDeviceConstants.DRIBBLER_MAX_SPEED,
        )

    def __parse_kick_event_value(self, value: float) -> float:
        return numpy.clip(
            a=self.power_control.chicker.kick_speed_m_per_s + value * HandheldDeviceConstants.KICK_POWER_STEPPER,
            a_min=HandheldDeviceConstants.MIN_KICK_POWER,
            a_max=HandheldDeviceConstants.MAX_KICK_POWER,
        )

    def __parse_chip_event_value(self, value: float) -> float:
        return numpy.clip(
            a=value * HandheldDeviceConstants.CHIP_DISTANCE_STEPPER,
            a_min=HandheldDeviceConstants.MIN_CHIP_POWER,
            a_max=HandheldDeviceConstants.MAX_CHIP_POWER,
        )

# TODO: remove thee after field testing...
# {
#   ('EV_SYN', 0): [('SYN_REPORT', 0), ('SYN_CONFIG', 1), ('SYN_DROPPED', 3), ('?', 21)],
#   ('EV_KEY', 1): [
#     ('KEY_RECORD', 167),
#     (['BTN_A', 'BTN_GAMEPAD', 'BTN_SOUTH'], 304),
#     (['BTN_B', 'BTN_EAST'], 305),
#     (['BTN_NORTH', 'BTN_X'], 307),
#     (['BTN_WEST', 'BTN_Y'], 308),
#     ('BTN_TL', 310),
#     ('BTN_TR', 311),
#     ('BTN_SELECT', 314),
#     ('BTN_START', 315),
#     ('BTN_MODE', 316),
#     ('BTN_THUMBL', 317),
#     ('BTN_THUMBR', 318)
#   ],
#   ('EV_ABS', 3): [
#     (('ABS_X', 0), AbsInfo(value=1242, min=-32768, max=32767, fuzz=16, flat=128, resolution=0)),
#     (('ABS_Y', 1), AbsInfo(value=425, min=-32768, max=32767, fuzz=16, flat=128, resolution=0)),
#     (('ABS_Z', 2), AbsInfo(value=0, min=0, max=1023, fuzz=0, flat=0, resolution=0)),
#     (('ABS_RX', 3), AbsInfo(value=-418, min=-32768, max=32767, fuzz=16, flat=128, resolution=0)),
#     (('ABS_RY', 4), AbsInfo(value=-485, min=-32768, max=32767, fuzz=16, flat=128, resolution=0)),
#     (('ABS_RZ', 5), AbsInfo(value=0, min=0, max=1023, fuzz=0, flat=0, resolution=0)),
#     (('ABS_HAT0X', 16), AbsInfo(value=0, min=-1, max=1, fuzz=0, flat=0, resolution=0)),
#     (('ABS_HAT0Y', 17), AbsInfo(value=0, min=-1, max=1, fuzz=0, flat=0, resolution=0))
#   ],
#   ('EV_FF', 21): [
#     (['FF_EFFECT_MIN', 'FF_RUMBLE'], 80),
#     ('FF_PERIODIC', 81),
#     (['FF_SQUARE', 'FF_WAVEFORM_MIN'], 88),
#     ('FF_TRIANGLE', 89),
#     ('FF_SINE', 90),
#     (['FF_GAIN', 'FF_MAX_EFFECTS'], 96)
#   ]
# }
