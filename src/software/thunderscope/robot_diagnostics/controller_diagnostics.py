import time

import numpy
from multiprocessing import Process
from evdev import InputDevice, categorize, ecodes, list_devices, InputEvent

import software.python_bindings as tbots_cpp
from proto.import_all_protos import *
from software.thunderscope.constants import *
from software.thunderscope.constants import ControllerConstants
from software.thunderscope.robot_diagnostics.controller_status_view import (
    ControllerConnectionState,
)
from software.thunderscope.robot_diagnostics.diagnostics_input_widget import ControlMode


# TODO: the following is logged on controller connection during runtime:
# `qt.qpa.input.events: scroll event from unregistered device 17`


class MoveEventType(Enum):
    LINEAR = 1
    ROTATIONAL = 2


class ControllerHandler(object):
    """
    This class is responsible for reading from a handheld controller device and
    interpreting the device inputs into usable inputs for the robots.
    """

    def __init__(self, logger):
        self.logger = logger
        self.controller = None
        self.controller_codes = None

        # Control primitives that are directly updated with
        # parsed controller inputs on every iteration of the event loop
        self.motor_control = MotorControl()
        self.power_control = PowerControl()

        # These fields are here to temporarily persist the controller's input.
        # They are read once once certain buttons are pressed on the controller,
        # and the values are inserted into the control primitives above.
        self.kick_power_accumulator = 0
        self.dribbler_speed_accumulator = 0

        self.constants = tbots_cpp.create2021RobotConstants()

        # event that is used to stop the controller event loop
        # self.__stop_thread_signal_event = Event()

        # Set-up process that will run the event loop
        self.__setup_new_event_listener_process()

        self.initialize_controller()

        if (
            self.get_controller_connection_status()
            == ControllerConnectionState.CONNECTED
        ):

            self.logger.debug(
                "Initialized handheld controller "
                + '"'
                + self.controller.name
                + '"'
                + " and located at path: "
                + self.controller.path
            )

        elif (
            self.get_controller_connection_status()
            == ControllerConnectionState.DISCONNECTED
        ):
            self.logger.debug(
                "Failed to initialize a handheld controller, check USB connection"
            )
            self.logger.debug("Tried the following available devices:")
            self.logger.debug(
                list(map(lambda device: InputDevice(device).name, list_devices()))
            )

    def get_controller_connection_status(self) -> ControllerConnectionState:
        return (
            ControllerConnectionState.CONNECTED
            if self.controller is not None
            else ControllerConnectionState.DISCONNECTED
        )

    def refresh(self, mode: ControlMode):
        """
        Refresh this class.
        Spawns a new process that listens and processes
        handheld controller device events if Control.Mode is Diagnostics,
        Otherwise, terminates the current listener process if it is running,
        to avoid busy waiting
        :param mode: The current user requested mode for controlling the robot
        """
        if mode == ControlMode.DIAGNOSTICS:
            self.logger.debug("Terminating controller event handling process")
            if self.__controller_event_loop_handler_process.is_alive():
                # self.__stop_thread_signal_event.set()
                self.__controller_event_loop_handler_process.terminate()

        elif mode == ControlMode.HANDHELD:
            self.logger.debug("Setting up new controller event handling process")
            self.__setup_new_event_listener_process()
            self.__start_event_listener_process()

    def get_latest_primitive_controls(self):
        primitive = DirectControlPrimitive(
            motor_control=self.motor_control, power_control=self.power_control
        )
        # TODO: pre-emptive bugfix: need to reset controls, epecially power so that
        #  the control message isn't set to what is essentially auto-kick/chip
        # self.motor_control = MotorControl()
        self.power_control = PowerControl()
        return primitive

    def initialize_controller(self):
        """
        Attempt to initialize a controller.
        The first controller that is recognized a valid controller will be used
        """
        for device in list_devices():
            controller = InputDevice(device)
            if (
                controller is not None
                and controller.name in ControllerConstants.CONTROLLER_NAME_CODES_MAP
            ):
                self.controller = controller
                self.controller_codes = ControllerConstants.CONTROLLER_NAME_CODES_MAP[
                    controller.name
                ]
                break

    def __start_event_listener_process(self):
        # start the process for reading controller events
        if not self.__controller_event_loop_handler_process.is_alive():
            self.__controller_event_loop_handler_process.start()

    def __setup_new_event_listener_process(self):
        """
        initializes a new process that will run the event processing loop
        """

        # TODO (#3165): Use trace level self.logger here
        # self.logger.debug("Starting controller event loop process")
        self.__controller_event_loop_handler_process = Process(
            target=self.__event_loop, daemon=True
        )

    def close(self):
        """
        Shut down the controller event loop
        :return:
        """
        # TODO (#3165): Use trace level logging here
        # self.logger.debug("Shutdown down controller event loop process")
        # self.__stop_thread_signal_event.set()
        self.__controller_event_loop_handler_process.terminate()

    def __event_loop(self):
        # TODO (#3165): Use trace level self.logger here
        self.logger.debug("Starting handheld controller event handling loop")
        try:
            for event in self.controller.read_loop():
                # TODO: is this a useful comment?
                # All events accumulate into a file and will be read back eventually,
                # even if handheld mode is disabled. This time check ensures that
                # only the events that have occurred very recently are processed, and
                # any events that occur before switching to handheld mode are ignored
                if (
                    time.time() - event.timestamp()
                    < ControllerConstants.INPUT_DELAY_THRESHOLD
                ):
                    self.__process_event(event)
        except OSError as ose:
            self.logger.debug(
                "Caught an OSError while reading handheld controller event loop!"
            )
            self.logger.debug("Error message: " + str(ose))
            self.logger.debug("Check physical handheld controller USB connection!")
            return
        except Exception as e:
            self.logger.critical(
                "Caught an unexpected error while reading handheld controller event loop!"
            )
            self.logger.critical("Error message: " + str(e))
            return

    def __process_event(self, event: InputEvent):

        # abs_event = categorize(event)
        event_type = ecodes.bytype[event.type][event.code]

        # if event.type == ecodes.EV_ABS or event.type == ecodes.EV_KEY:
        # TODO (#3165): Use trace level self.logger here
        self.logger.debug(
            "Processing controller event with type: "
            + str(event_type)
            + ", with code: "
            + str(event.code)
            + ", and with value: "
            + str(event.value)
        )

        if event.code == self.controller_codes[InputEventType.MOVE_X]:
            self.motor_control.direct_velocity_control.velocity.x_component_meters = self.__parse_move_event_value(
                ControllerConstants.MAX_LINEAR_SPEED_METER_PER_S, event.value
            )

        if event.code == self.controller_codes[InputEventType.MOVE_Y]:
            self.motor_control.direct_velocity_control.velocity.y_component_meters = self.__parse_move_event_value(
                ControllerConstants.MAX_LINEAR_SPEED_METER_PER_S, event.value
            )

        if event.code == self.controller_codes[InputEventType.ROTATE]:
            self.motor_control.direct_velocity_control.angular_velocity.radians_per_second = self.__parse_move_event_value(
                ControllerConstants.MAX_ANGULAR_SPEED_RAD_PER_S, event.value
            )

        if event.code == self.controller_codes[InputEventType.KICK_POWER]:
            self.kick_power_accumulator = self.__parse_kick_event_value(event.value)

        if event.code == self.controller_codes[InputEventType.DRIBBLER_SPEED]:
            self.dribbler_speed_accumulator = self.__parse_dribble_event_value(
                event.value
            )

        # Left and right triggers
        if (
            event.code == self.controller_codes[InputEventType.DRIBBLER_ENABLE_1]
            or event.code == self.controller_codes[InputEventType.DRIBBLER_ENABLE_2]
        ):
            dribbler_enabled = self.__parse_dribbler_enabled_event_value(event.value)
            if dribbler_enabled:
                self.motor_control.dribbler_speed_rpm = self.dribbler_speed_accumulator

        # "A" button
        if (
            event.code == self.controller_codes[InputEventType.KICK]
            and event.value == 1
        ):
            self.power_control.geneva_slot = 3
            self.power_control.chicker.kick_speed_m_per_s = self.kick_power_accumulator

        # "A" button
        if (
            event.code == self.controller_codes[InputEventType.CHIP]
            and event.value == 1
        ):
            self.power_control.geneva_slot = 3
            self.power_control.chicker.chip_distance_meters = (
                self.kick_power_accumulator
            )

    @staticmethod
    def __parse_move_event_value(event_value: float, scaling_factor: float) -> float:
        if abs(event_value) < (
            ControllerConstants.DEADZONE_PERCENTAGE * scaling_factor
        ):
            return 0
        else:
            return numpy.clip(
                event_value, 0, ControllerConstants.XBOX_MAX_RANGE * scaling_factor
            )

    @staticmethod
    def __parse_dribbler_enabled_event_value(value: float) -> bool:
        return value > (ControllerConstants.XBOX_BUTTON_MAX_RANGE / 2.0)

    @staticmethod
    def __parse_dribble_event_value(value: float) -> float:
        return numpy.clip(
            value * ControllerConstants.DRIBBLER_STEPPER,
            0,
            ControllerConstants.DRIBBLER_INDEFINITE_SPEED,
        )

    @staticmethod
    def __parse_kick_event_value(value: float) -> float:
        return numpy.clip(
            value * ControllerConstants.POWER_STEPPER,
            ControllerConstants.MIN_POWER,
            ControllerConstants.MAX_POWER,
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
