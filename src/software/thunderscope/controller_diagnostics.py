from proto.import_all_protos import *
from software.thunderscope.constants import *

from xbox360controller import Xbox360Controller

AXIS_DEAD_ZONE_THRESHOLD = 0.05
XY_SPEED_LIMIT_MS = 2.0
ANGULAR_SPEED_LIMIT_RAD_PER_S = 10.0

AUTO_KICK_SPEED_MS = 6.0
AUTO_KICK_ON_RUMBLE_LENGTH_MS = 1500
AUTO_KICK_OFF_RUMBLE_LENGTH_MS = 500

class ControllerDiagnostics(object):

    def __init__(self, proto_unix_io):
        self.controller = Xbox360Controller(0, axis_threshold=0.001)
        self.controller.axis_l.when_moved = self.__on_left_axis_moved
        self.controller.axis_r.when_moved = self.__on_right_axis_moved
        self.controller.trigger_r.when_moved = self.__on_right_trigger_moved
        self.controller.trigger_l.when_moved = self.__on_left_trigger_moved
        self.controller.button_a.when_pressed = self.__on_button_a_pressed
        self.controller.button_b.when_pressed = self.__on_button_b_pressed

        self.motor_control = MotorControl()
        self.proto_unix_io = proto_unix_io
        self.auto_kick_enabled = False

    def __on_left_axis_moved(self, axis):
        x = axis.x if abs(axis.x) > AXIS_DEAD_ZONE_THRESHOLD else 0
        y = axis.y if abs(axis.y) > AXIS_DEAD_ZONE_THRESHOLD else 0

        self.motor_control.direct_velocity_control.velocity.x_component_meters = -y * XY_SPEED_LIMIT_MS
        self.motor_control.direct_velocity_control.velocity.y_component_meters = -x * XY_SPEED_LIMIT_MS
        self.proto_unix_io.send_proto(MotorControl, self.motor_control)
        print(self.motor_control)

    def __on_right_axis_moved(self, axis):
        x = axis.x if abs(axis.x) > AXIS_DEAD_ZONE_THRESHOLD else 0

        self.motor_control.direct_velocity_control.angular_velocity.radians_per_second = -x * ANGULAR_SPEED_LIMIT_RAD_PER_S
        self.proto_unix_io.send_proto(MotorControl, self.motor_control)
        print(self.motor_control)

    def __on_right_trigger_moved(self, axis):
        self.motor_control.dribbler_speed_rpm = int(axis.value * MAX_DRIBBLER_RPM)
        self.proto_unix_io.send_proto(MotorControl, self.motor_control)
        print(self.motor_control)

    def __on_left_trigger_moved(self, axis):
        self.motor_control.dribbler_speed_rpm = int(-axis.value * MAX_DRIBBLER_RPM)
        self.proto_unix_io.send_proto(MotorControl, self.motor_control)
        print(self.motor_control)

    def __on_button_a_pressed(self, button):
        power_control = PowerControl()
        power_control.geneva_slot = 1
        power_control.chicker.auto_chip_or_kick.autokick_speed_m_per_s = AUTO_KICK_SPEED_MS
        self.proto_unix_io.send_proto(PowerControl, power_control)
        self.controller.set_rumble(1.0, 1.0, AUTO_KICK_ON_RUMBLE_LENGTH_MS)

    def __on_button_b_pressed(self, button):
        # Don't populate the autokick field, so the robot will stop auto kicking
        power_control = PowerControl()
        power_control.geneva_slot = 1
        self.proto_unix_io.send_proto(PowerControl, power_control)
        self.controller.set_rumble(0.3, 0.3, AUTO_KICK_OFF_RUMBLE_LENGTH_MS)

    def chip(self):
        power_control = PowerControl()
        power_control.geneva_slot = 1
        power_control.chicker.auto_chip_or_kick.autochip_distance_meters = ( self.power )
        
        self.proto_unix_io.send_proto(PowerControl, power_control)
        print(power_control)

