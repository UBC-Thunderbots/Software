import logging
from multiprocessing import Process

import numpy
from evdev import InputDevice, categorize, ecodes, list_devices
from threading import Event

import software.python_bindings as tbots_cpp
from proto.import_all_protos import *
from software.thunderscope.constants import *
from software.thunderscope.constants import ControllerConstants
from software.thunderscope.robot_diagnostics.diagnostics_input_widget import ControlMode


# TODO:
# following is logged on controller connection during runtime:
# `qt.qpa.input.events: scroll event from unregistered device 17`


class MoveEventType(Enum):
    LINEAR = 1
    ROTATIONAL = 2


class ControllerInputHandler(object):
    """
    This class is responsible for reading from a handheld controller device and
    interpreting the device inputs into usable inputs for the robots.
    """

    def __init__(self,):
        self.controller = None
        self.__controller_event_loop_handler_process = None

        self.motor_control = MotorControl()
        self.power_control = PowerControl()

        self.__setup_controller()

        if self.controller_initialized():

            logging.debug(
                "Initialized handheld controller "
                + '"'
                + self.controller.name
                + '"'
                + " and located at path: "
                + self.controller.path
            )

            self.constants = tbots_cpp.create2021RobotConstants()

            # event that is used to stop the controller event loop
            self.__stop_thread_signal_event = Event()

            self.__setup_new_event_listener_process()

        elif self.controller_initialized():
            logging.debug(
                "Tried to initialize a handheld controller from list available devices:"
            )
            logging.debug(
                list(map(lambda device: InputDevice(device).name, list_devices()))
            )
            logging.debug(
                "Could not initialize a handheld controller device - check USB connections"
            )

    def controller_initialized(self) -> bool:
        return self.controller is not None

    def toggle_controller_listener(self, mode: ControlMode):
        """
        Changes the diagnostics input mode for all robots between Xbox and Diagnostics.

        :param mode: current mode of control.
        """
        if mode == ControlMode.DIAGNOSTICS:
            logging.debug("Terminating controller event handling process")
            self.__controller_event_loop_handler_process.terminate()
            self.__controller_event_loop_handler_process = None

        elif mode == ControlMode.HANDHELD:
            logging.debug("Setting up new controller event handling process")
            # self.__setup_event_listener_process()
            self.__setup_new_event_listener_process()
            self.__start_event_listener_process()

    def get_latest_primitive_command(self):
        if self.controller_initialized():
            return DirectControlPrimitive(
                motor_control=self.motor_control, power_control=self.power_control,
            )
        else:
            return None

    def __setup_controller(self):
        for device in list_devices():
            controller = InputDevice(device)
            if (
                controller is not None
                and controller.name in ControllerConstants.VALID_CONTROLLERS
            ):
                self.controller = controller
                break

    def __start_event_listener_process(self):
        # start the process for reading controller events
        if not self.__controller_event_loop_handler_process.is_alive():
            self.__controller_event_loop_handler_process.start()

    def __setup_new_event_listener_process(self):
        """
        initializes & starts a new process that runs the event processing loop
        """

        # # TODO (#3165): Use trace level logging here
        # logging.debug("Starting controller event loop process")
        if self.__controller_event_loop_handler_process is None:
            self.__controller_event_loop_handler_process = (
                Process(target=self.__event_loop, daemon=True)
            )

    def close(self):
        # # TODO (#3165): Use trace level logging here
        self.__stop_thread_signal_event.set()
        self.__controller_event_loop_handler_process.join()

    def __event_loop(self):
        # # TODO (#3165): Use trace level logging here
        # logging.debug("Starting handheld controller event handling loop")
        try:
            for event in self.controller.read_loop():
                if self.__stop_thread_signal_event.isSet():
                    return
                else:
                    self.__process_event(event)

        except OSError as ose:
            logging.debug(
                "Caught an OSError while reading handheld controller event loop!"
            )
            logging.debug("Error message: " + str(ose))
            logging.debug("Check physical handheld controller USB connection")
            return
        except Exception as e:
            logging.critical(
                "Caught an unexpected error while reading handheld controller event loop!"
            )
            logging.critical("Error message: " + str(e))
            return

    def __process_move_event_value(self, event_type, event_value) -> None:
        if event_type == "ABS_X":
            self.motor_control.direct_velocity_control.velocity.x_component_meters = self.__parse_move_event_value(
                MoveEventType.LINEAR, event_value
            )

        elif event_type == "ABS_Y":
            self.motor_control.direct_velocity_control.velocity.y_component_meters = (
                self.__parse_move_event_value(
                    MoveEventType.LINEAR, event_value
                )
            )

        elif event_type == "ABS_RX":
            self.motor_control.direct_velocity_control.angular_velocity.radians_per_second = (
                self.__parse_move_event_value(
                    MoveEventType.ROTATIONAL, event_value
                )
            )

    def __process_event(self, event):
        kick_power = 0.0
        dribbler_speed = 0.0

        abs_event = categorize(event)
        event_type = ecodes.bytype[abs_event.event.type][abs_event.event.code]

        # # TODO (#3165): Use trace level logging here
        # logging.debug(
        #     "Processing controller event with type "
        #     + str(event_type)
        #     + " and with value "
        #     + str(abs_event.event.value)
        # )

        # TODO (#3175): new python versions have a pattern matching,
        #  that could be useful for handling lots of if statements.
        #  Some documentation for future:
        #  https://peps.python.org/pep-0636/
        #  https://docs.python.org/3/reference/compound_stmts.html#match
        if event.type == ecodes.EV_ABS:
            if event_type in ["ABS_X", "ABS_Y", "ABS_RX"]:
                self.__process_move_event_value(event_type, abs_event.event.value)

        if event_type == "ABS_HAT0X":
            dribbler_speed = self.__parse_kick_event_value(abs_event.event.value)

        if event_type == "ABS_HAT0Y":
            kick_power = self.__parse_dribble_event_value(abs_event.event.value)

        if event_type == "ABS_RZ" or "ABS_Z":
            if self.__parse_dribbler_enabled_event_value(abs_event.event.value):
                self.motor_control.dribbler_speed_rpm = float(dribbler_speed)

        if event.type == ecodes.EV_KEY:
            # TODO: try testing event_type instead of checking in `ecodes.ecodes` map
            if event.code == ecodes.ecodes["BTN_A"] and event.value == 1.0:
                self.power_control.geneva_slot = 3.0
                self.power_control.chicker.kick_speed_m_per_s = kick_power

            elif event.code == ecodes.ecodes["BTN_Y"] and event.value == 1.0:
                self.power_control.geneva_slot = 3.0
                self.power_control.chicker.chip_distance_meters = kick_power

    @staticmethod
    def __parse_move_event_value(
        event_type: MoveEventType, event_value: float
    ) -> float:
        factor = (
            ControllerConstants.MAX_ANGULAR_SPEED_RAD_PER_S
            if event_type == MoveEventType.ROTATIONAL
            else ControllerConstants.MAX_LINEAR_SPEED_METER_PER_S
            if event_type == MoveEventType.LINEAR
            else 1.0
        )

        if abs(event_value) < (ControllerConstants.DEADZONE_PERCENTAGE * factor):
            return 0
        else:
            return numpy.clip(
                event_value, 0, ControllerConstants.XBOX_MAX_RANGE * factor
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
