import logging
from evdev import InputDevice, categorize, ecodes, list_devices
from threading import Event, Thread

import software.python_bindings as tbots_cpp
from proto.import_all_protos import *
from software.thunderscope.constants import *
from software.thunderscope.proto_unix_io import ProtoUnixIO


# TODO: change all logging to DEBUG level or remove entirely...


class ControllerInputHandler(object):
    """
    This class is responsible for reading from an Xbox controller device and
    interpreting the inputs into usable inputs for robot.
    """

    # TODO: remove proto_unix_io, and set Motor/Power control as class fields
    # TODO: add class init wrapper for easier handling of controller connection
    def __init__(
        self, proto_unix_io: ProtoUnixIO,
    ):
        self.proto_unix_io = proto_unix_io
        self.enabled = False
        self.controller = None

        self.__setup_controller()

        if self.controller is not None:
            logging.debug(
                "Initializing controller "
                + self.controller.info.__str__()
                + " and device path location: "
                + self.controller.path
            )

            self.__stop_event_thread = Event()
            self.__event_thread = Thread(target=self.__event_loop, daemon=True)
            self.__event_thread.start()

            self.constants = tbots_cpp.create2021RobotConstants()

            self.x_vel = 0.0
            self.y_vel = 0.0
            self.ang_vel = 0.0
            self.kick_power = 0.0
            self.dribbler_speed = 0.0
            self.dribbler_enabled = False

            self.motor_control = MotorControl()
            self.power_control = PowerControl()

        else:
            logging.debug(
                "Tried to initialize a handheld controller from list available devices:"
            )
            logging.debug(list_devices())
            logging.debug(
                "Could not initialize a handheld controller device - check USB connections"
            )

    def get_latest_primitive_command(self):
        if self.controller_initialized():
            return DirectControlPrimitive(
                motor_control=self.motor_control, power_control=self.power_control,
            )
        else:
            return None

    def controller_initialized(self):
        return self.controller is not None

    def __setup_controller(self):
        for device in list_devices():
            controller = InputDevice(device)
            if (
                controller is not None
                and controller.name in ControllerConstants.VALID_CONTROLLER_NAMES
            ):
                self.controller = controller
                break

    @staticmethod
    def __parse_dribbler_enabled_event_value(value) -> bool:
        return value > (XBOX_BUTTON_MAX_RANGE / 2)

    @staticmethod
    def __parse_move_event_value(value, factor):
        if abs(value) < (DEADZONE_PERCENTAGE * factor):
            return 0
        else:
            return value / (XBOX_MAX_RANGE * factor)

    def __parse_dribbler_event_value(self, value):
        # can probably combine these 2 parsers but abastracting the +/- is annoying so 2 funcs it is
        new_speed = self.dribbler_speed - value * DRIBBLER_STEPPER
        if MIN_DRIBBLER_RPM < new_speed < MAX_DRIBBLER_RPM:
            return new_speed
        else:
            return 0

    def __parse_kick_event_value(self, value) -> int:
        new_power = self.kick_power + value * POWER_STEPPER
        if MIN_POWER < new_power < MAX_POWER:
            return new_power
        else:
            return 0

    def __process_event(self, event):
        logging.debug("Processing event: " + str(event))

        abs_event = categorize(event)

        # TODO: bump python version so we can use pattern matching for this
        if event.type == ecodes.EV_ABS:
            # grab the event type
            event_t = ecodes.bytype[abs_event.event.type][abs_event.event.code]
            logging.debug("Processing joystick event type: " + str(event_t))

            if event_t == "ABS_X":
                self.x_vel = self.__parse_move_event_value(
                    abs_event.event.value, MAX_LINEAR_SPEED_METER_PER_S
                )

            elif event_t == "ABS_Y":
                self.y_vel = self.__parse_move_event_value(
                    abs_event.event.value, MAX_LINEAR_SPEED_METER_PER_S
                )

            elif event_t == "ABS_RX":
                self.ang_vel = self.__parse_move_event_value(
                    abs_event.event.value, MAX_ANGULAR_SPEED_RAD_PER_S
                )

            elif event_t == "ABS_HAT0Y":
                self.dribbler_speed = self.__parse_dribbler_event_value(
                    abs_event.event.value
                )

            elif event_t == "ABS_HAT0X":
                self.kick_power = self.__parse_kick_event_value(abs_event.event.value)

            elif event_t == "ABS_RZ":
                self.dribbler_enabled = self.__parse_dribbler_enabled_event_value(
                    abs_event.event.value
                )

            elif event_t == "ABS_Z":
                self.dribbler_enabled = self.__parse_dribbler_enabled_event_value(
                    abs_event.event.value
                )

        elif event.type == ecodes.EV_KEY:
            logging.debug("Processing button event: " + str(event.code))
            if event.code == ecodes.ecodes["BTN_A"] and event.value == 1:
                self.power_control.geneva_slot = 1
                self.power_control.chicker.kick_speed_m_per_s = self.kick_power

            elif event.code == ecodes.ecodes["BTN_Y"] and event.value == 1:
                self.power_control.geneva_slot = 1
                # TODO: not sure if we should use this value
                self.power_control.chicker.chip_distance_meters = self.kick_power

        if event.type in ["ABS_X", "ABS_Y", "ABS_RX"]:
            (
                self.motor_control.direct_velocity_control.velocity
            ).x_component_meters = self.x_vel
            (
                self.motor_control.direct_velocity_control.velocity
            ).y_component_meters = self.y_vel
            (
                self.motor_control.direct_velocity_control.angular_velocity
            ).radians_per_second = self.ang_vel

            self.motor_control.dribbler_speed_rpm = 0

            # if self.enable_dribbler:
            #     motor_control.dribbler_speed_rpm = dribbler_speed
            # TODO: maybe just use indefinite instead? or
            #   have setting to turn on 'smooth scrolling'
            if self.dribbler_enabled:
                self.motor_control.dribbler_speed_rpm = (
                    self.constants.indefinite_dribbler_speed_rpm
                )

    def __event_loop(self):
        logging.debug("Starting handheld controller event loop")
        if self.enabled:
            for event in self.controller.read_loop():
                # TODO: only run loop if self.enabled is set
                if self.__stop_event_thread.isSet():
                    return
                else:
                    self.__process_event(event)

    def close(self):
        logging.debug("Closing controller thread")
        self.__stop_event_thread.set()
        self.__event_thread.join()

    def set_enabled(self, enabled: bool):
        """
        Changes the diagnostics input mode for all robots between Xbox and Diagnostics.

        :param enabled: to which state to set controller enabled.
        """
        self.enabled = enabled


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
