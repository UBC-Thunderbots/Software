from evdev import InputDevice, categorize, ecodes

XBOX_MAX_RANGE = 32768

class XboxDiagnostics(object):

    def __init__(self, input_path, proto_unix_io):
        self.controller = InputDevice(input_path)

        self.stop_event_thread = Event()
        self._event_thread = Thread(target=self._event_loop)
        self._event_thread.start()

        self.move_x = 0;
        self.move_y = 0;

    def _event_loop(self) :
        for event in controller.read_loop() :
            if (self.stop_event_thread.is_set())
                return
            self.process_event(event)

    def process_event(self, event) :
        if (event.type == ecodes.EV_ABS) :
           absevent = categorize(event)
           if (ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_X"):
               self.move_x = absevent.event.value
               self.update_move_primitive()
            elif (ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_Y"):
                self.move_y = absevent.event.value / 32768
                self.update_move_primitive()

    def update_move_primitive(self):
        motor_control = MotorControl()
        moto_control.direct_velocity_control.


    def close(self):
        self.stop_event_thread.set()
        self._event_thread.join()
