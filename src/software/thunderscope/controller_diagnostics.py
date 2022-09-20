from proto.import_all_protos import *
from software.thunderscope.constants import *

from evdev import InputDevice, categorize, ecodes
from threading import Event, Thread

XBOX_MAX_RANGE = 32768

class ControllerDiagnostics(object):

    def __init__(self, input_path, proto_unix_io):
        self.controller = InputDevice(input_path)
        print("Start input device " + input_path)
        self.proto_unix_io = proto_unix_io

        self.stop_event_thread = Event()
        self._event_thread = Thread(target=self._event_loop)
        self._event_thread.start()

        self.move_x = 0;
        self.move_y = 0;

    def _event_loop(self) :
        print("start event loop")
        for event in self.controller.read_loop() :
            if (self.stop_event_thread.isSet()) :
                return
            self.process_event(event)

    def process_event(self, event) :
        if (event.type == ecodes.EV_ABS) :
            absevent = categorize(event)
            if (ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_X"):
                self.move_x = absevent.event.value / XBOX_MAX_RANGE * MAX_LINEAR_SPEED_MPS
                self.update_move_primitive()
            elif (ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_Y"):
                self.move_y = -absevent.event.value / XBOX_MAX_RANGE * MAX_LINEAR_SPEED_MPS
                self.update_move_primitive()

    def update_move_primitive(self):
        motor_control = MotorControl()

        motor_control.dribbler_speed_rpm = 0
        motor_control.direct_velocity_control.velocity.x_component_meters = ( self.move_x )
        motor_control.direct_velocity_control.velocity.y_component_meters = ( self.move_y )
        motor_control.direct_velocity_control.angular_velocity.radians_per_second = ( 0 )

        self.proto_unix_io.send_proto(MotorControl, motor_control)

    def close(self):
        self.stop_event_thread.set()
        self._event_thread.join()
