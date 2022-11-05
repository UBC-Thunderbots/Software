from proto.import_all_protos import *
from software.thunderscope.constants import *

from evdev import InputDevice, categorize, ecodes
from threading import Event, Thread

XBOX_MAX_RANGE = 32768
XBOX_BUTTON_MAX_RANGE = 1024

DEADZONE_PERCENTAGE = 0.30

DRIBBLER_STEPPER = 100
POWER_STEPPER = 100

MAX_LINEAR_SPEED_MPS = 2

class ControllerDiagnostics(object):
    def __init__(self, input_path, proto_unix_io):
        self.controller = InputDevice(input_path)
        print("Start input device " + input_path)
        self.proto_unix_io = proto_unix_io

        self.stop_event_thread = Event()
        self._event_thread = Thread(target=self._event_loop)
        self._event_thread.start()

        self.move_x = 0
        self.move_y = 0
        self.ang_vel = 0
        self.power = 1000

        self.enable_dribbler = 0
        self.dribbler_speed = 0

    def _event_loop(self):
        print("start event loop")
        for event in self.controller.read_loop():
            if self.stop_event_thread.isSet():
                return
            self.process_event(event)

    def process_event(self, event):
        if event.type == ecodes.EV_ABS:
            absevent = categorize(event)
            event_t = ecodes.bytype[absevent.event.type][absevent.event.code]
            print("EVENT: " + event_t)
            if event_t == "ABS_X":
                self.update_move(
                    x=None,
                    y=absevent.event.value / XBOX_MAX_RANGE * MAX_LINEAR_SPEED_MPS,
                )
            elif event_t == "ABS_Y":
                self.update_move(
                    x=-absevent.event.value / XBOX_MAX_RANGE * MAX_LINEAR_SPEED_MPS,
                    y=None,
                )
            elif event_t == "ABS_RX":
                self.update_angular_velocity(
                    absevent.event.value / XBOX_MAX_RANGE * MAX_ANGULAR_SPEED_RAD_PER_S
                )
            elif event_t == "ABS_HAT0Y":
                self.update_dribbler(
                    self.dribbler_speed - absevent.event.value * DRIBBLER_STEPPER
                )
            elif event_t == "ABS_HAT0X":
                self.update_power(self.power + absevent.event.value * POWER_STEPPER)
            elif event_t == "ABS_RZ":
                if absevent.event.value > (XBOX_BUTTON_MAX_RANGE / 2):
                    self.enable_dribbler = 1
                    print("dribbler enabled")
            elif event_t == "ABS_Z":
                if absevent.event.value < (XBOX_BUTTON_MAX_RANGE / 2):
                    self.enable_dribbler = 0
                    print("dribbler enabled")


            if (
                event_t == "ABS_X"
                or event_t == "ABS_Y"
                or event_t == "ABS_RX"
                or event_t == "ABS_HAT0Y"
                or event_t == "ABS_RZ"
            ):
                self.send_move_primitive()
        elif event.type == ecodes.EV_KEY:
            print("event code: " + str(event.code))
            if event.code == ecodes.ecodes["BTN_A"] and event.value == 1:
                print("kick")
                self.kick()
            elif event.code == ecodes.ecodes["BTN_Y"] and event.value == 1:
                print("chip")
                self.chip()

    def kick(self):
        power_control = PowerControl()
        power_control.geneva_slot = 1
        power_control.chicker.kick_speed_m_per_s = self.power

        self.proto_unix_io.send_proto(PowerControl, power_control)
        print(power_control)

    def chip(self):
        power_control = PowerControl()
        power_control.geneva_slot = 1
        power_control.chicker.chip_distance_meters = self.power

        self.proto_unix_io.send_proto(PowerControl, power_control)
        print(power_control)

    def update_power(self, new_power):
        if new_power > MIN_POWER and new_power < MAX_POWER:
            self.power = new_power
        print("power: " + str(self.power))

    def update_dribbler(self, new_dribbler_speed):
        if (
            new_dribbler_speed < MAX_DRIBBLER_RPM
            and new_dribbler_speed > MIN_DRIBBLER_RPM
        ):
            self.dribbler_speed = new_dribbler_speed
        print("dribbler speed: " + str(self.dribbler_speed))

    def update_move(self, x, y):
        if x is not None:
            self.move_x = x
        if y is not None:
            self.move_y = y

        if (abs(self.move_x) < (DEADZONE_PERCENTAGE * MAX_LINEAR_SPEED_MPS)) :
            self.move_x = 0
        if (abs(self.move_y) < (DEADZONE_PERCENTAGE * MAX_LINEAR_SPEED_MPS)) :
            self.move_y = 0

        self.ang_vel = 0

    def update_angular_velocity(self, ang_vel):
        self.ang_vel = ang_vel if ang_vel > (MAX_ANGULAR_SPEED_RAD_PER_S * DEADZONE_PERCENTAGE) else 0
        self.move_x = 0
        self.move_y = 0
        print("new and vel: " + str(self.ang_vel))

    def send_move_primitive(self):
        motor_control = MotorControl()

        motor_control.dribbler_speed_rpm = 0
        if self.enable_dribbler:
            motor_control.dribbler_speed_rpm = self.dribbler_speed

        motor_control.direct_velocity_control.velocity.x_component_meters = self.move_x
        motor_control.direct_velocity_control.velocity.y_component_meters = self.move_y
        motor_control.direct_velocity_control.angular_velocity.radians_per_second = (
            self.ang_vel
        )

        print(motor_control)
        self.proto_unix_io.send_proto(MotorControl, motor_control)

    def close(self):
        self.stop_event_thread.set()
        self._event_thread.join()
