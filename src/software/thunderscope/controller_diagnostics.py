from glob import glob
from typing import Callable

import evdev
from evdev import InputDevice, categorize, ecodes
from threading import Event, Thread

from software.thunderscope.proto_unix_io import ProtoUnixIO
import software.python_bindings as tbots_cpp


XBOX_MAX_RANGE = 32768
XBOX_BUTTON_MAX_RANGE = 1024

DEADZONE_PERCENTAGE = 0.30

DRIBBLER_STEPPER = 100
POWER_STEPPER = 100

MAX_LINEAR_SPEED_MPS = 2


class ControllerDiagnostics(object):
    """
    This class is responsible for reading from an Xbox controller device and
    interpreting the inputs into usable inputs for robot.
    """

    # TODO add function descriptions
    def __init__(
            self,
            input_path: str,
            proto_unix_io: ProtoUnixIO,
    ):
        for device in evdev.list_devices():
            try:
                self.controller = InputDevice(device)
            except Exception as e:
                print("Failed to initialize device with path: " + input_path + " with exception " + e)

        # TODO check valid path
        # TODO add auto detect controller
        print("Initializing controller with device path: " + self.controller.path)
        self.proto_unix_io = proto_unix_io

        self.stop_event_thread = Event()
        self._event_thread = Thread(target=self._event_loop)
        self._event_thread.start()

        self.constants = tbots_cpp.create2021RobotConstants()

        # self.move_x = 0
        # self.move_y = 0
        # self.ang_vel = 0
        # self.power = 1000

        # self.enable_dribbler = 0
        # self.dribbler_speed = 0

    def _event_loop(self):
        print("Starting controller event loop")
        for event in self.controller.read_loop():
            if self.stop_event_thread.isSet():
                return
            self.process_event(event)

    def process_event(self, event):
        print("Processing event: " + event)
        if event.type == ecodes.EV_ABS:
            absevent = categorize(event)
            event_t = ecodes.bytype[absevent.event.type][absevent.event.code]
            # TODO remove as fields and pass as args to handlers

            x_vel = 0
            y_vel = 0
            ang_vel = 0
            kick_power = 0

            dribbler_enabled = 0

            if event_t == "ABS_X":
                x_vel = self.coerce_move_value(absevent.event.value, MAX_LINEAR_SPEED_MPS)
                self.update_move(
                    x=None,
                    y=absevent.event.value / XBOX_MAX_RANGE * MAX_LINEAR_SPEED_MPS,
                )

            elif event_t == "ABS_Y":
                y_vel = self.coerce_move_value(absevent.event.value, MAX_LINEAR_SPEED_MPS)
                self.update_move(
                    x=-absevent.event.value / XBOX_MAX_RANGE * MAX_LINEAR_SPEED_MPS,
                    y=None,
                )

            elif event_t == "ABS_RX":
                ang_vel = self.coerce_move_value(absevent.event.value, MAX_ANGULAR_SPEED_RAD_PER_S)
                self.update_angular_velocity(
                    absevent.event.value / XBOX_MAX_RANGE * MAX_ANGULAR_SPEED_RAD_PER_S
                )

            # elif event_t == "ABS_HAT0Y":
            #     self.update_dribbler(
            #         self.dribbler_speed - absevent.event.value * DRIBBLER_STEPPER
            #     )
            # elif event_t == "ABS_HAT0X":
            #     self.update_power(self.power + absevent.event.value * POWER_STEPPER)
            # elif event_t == "ABS_RZ":
            #     if absevent.event.value > (XBOX_BUTTON_MAX_RANGE / 2):
            #         self.enable_dribbler = 1
            #         print("dribbler enabled")
            # elif event_t == "ABS_Z":
            #     if absevent.event.value < (XBOX_BUTTON_MAX_RANGE / 2):
            #         self.enable_dribbler = 0
            #         print("dribbler enabled")

            if (
                    event_t == "ABS_X"
                    or event_t == "ABS_Y"
                    or event_t == "ABS_RX"
                    or event_t == "ABS_HAT0Y"
                    or event_t == "ABS_RZ"
            ):
                self.send_move_command(
                    dribbler_enabled,
                    x_vel,
                    y_vel,
                    ang_vel
                )
        elif event.type == ecodes.EV_KEY:
            if event.code == ecodes.ecodes["BTN_A"] and event.value == 1:
                print("kick")
                self.send_kick_command()
            elif event.code == ecodes.ecodes["BTN_Y"] and event.value == 1:
                print("chip")
                self.send_chip_command(1)

    def send_move_command(
            self, dribbler_enabled: int, x_vel: int, y_vel: int, ang_vel: int
    ):
        motor_control = MotorControl()

        motor_control.direct_velocity_control.velocity.x_component_meters = x_vel
        motor_control.direct_velocity_control.velocity.y_component_meters = y_vel
        motor_control.direct_velocity_control.angular_velocity.radians_per_second = ang_vel

        motor_control.dribbler_speed_rpm = 0

        # if self.enable_dribbler:
        #     motor_control.dribbler_speed_rpm = dribbler_speed
        if dribbler_enabled:
            motor_control.dribbler_speed_rpm = self.constants.indefinite_dribbler_speed_rpm

        print("Sending motor control: " + motor_control)

        self.proto_unix_io.send_proto(MotorControl, motor_control)

    def send_kick_command(self, power: int):
        power_control = PowerControl()
        power_control.geneva_slot = 1
        power_control.chicker.kick_speed_m_per_s = power

        print("Sending kick power control: " + power_control)

        self.proto_unix_io.send_proto(PowerControl, power_control)

    def send_chip_command(self, distance: int):
        power_control = PowerControl()
        power_control.geneva_slot = 1
        power_control.chicker.chip_distance_meters = distance

        print("Sending chip power control: " + power_control)

        self.proto_unix_io.send_proto(PowerControl, power_control)

    def update_power(self, new_power):
        if MIN_POWER < new_power < MAX_POWER:
            self.power = new_power
        print("power: " + str(self.power))

    def update_dribbler(self, new_dribbler_speed):
        if MAX_DRIBBLER_RPM > new_dribbler_speed > MIN_DRIBBLER_RPM:
            self.dribbler_speed = new_dribbler_speed
        print("dribbler speed: " + str(self.dribbler_speed))


    def coerce_move_value(self, value, factor):
        if abs(value) < (DEADZONE_PERCENTAGE * factor):
            return 0
        else:
            return value / (XBOX_MAX_RANGE * factor)

    def close(self):
        print("Closing controller thread")
        self.stop_event_thread.set()
        self._event_thread.join()

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
