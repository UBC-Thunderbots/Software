import Jetson.GPIO as GPIO

# Channel B and Channel A states
STATE_0 = [0, 0]
STATE_1 = [0, 1]
STATE_2 = [1, 1]
STATE_3 = [1, 0]

CLOCKWISE = 1
COUNTERCLOCKWISE = -1
IDLE = 0

class RotaryEncoder:
    def __init__(
        self, A_PIN, B_PIN, BUTTON_PIN, 
        on_clockwise_rotate, on_counterclockwise_rotate, on_click
    ):
        self.A_PIN = A_PIN
        self.B_PIN = B_PIN
        self.BUTTON_PIN = BUTTON_PIN

        self.on_clockwise_rotate = on_clockwise_rotate
        self.on_counterclockwise_rotate = on_counterclockwise_rotate
        self.on_click = on_click

        self.counts_per_rotation = 4

    def init(self):
        """ Initialize GPIO pins and rotary encoder state """
        GPIO.setmode(GPIO.BOARD)

        # These pins will need external pullups set up
        GPIO.setup(self.A_PIN, GPIO.IN) 
        GPIO.setup(self.B_PIN, GPIO.IN) 
        GPIO.setup(self.BUTTON_PIN, GPIO.IN)

        a_state = GPIO.input(self.A_PIN)
        b_state = GPIO.input(self.B_PIN)

        self.curr_state = [b_state, a_state]
        self.dir = IDLE

        self.count = 0

    # Channel B and A go through 4 states for a single rotation.
    # The following are values for clockwise rotation:
    #  Channel B Channel A
    #     0          0
    #     0          1
    #     1          1
    #     1          0
    def rot_state(self):
        """ FSM to determine rotation direction and count state transitions """
        a_state = GPIO.input(self.A_PIN)
        b_state = GPIO.input(self.B_PIN)
        curr_state = [b_state, a_state]

        # State transitions
        if self.curr_state == STATE_0:
            if curr_state == STATE_1:
                self.dir = CLOCKWISE
            elif curr_state == STATE_3:
                self.dir = COUNTERCLOCKWISE
        elif self.curr_state == STATE_1:
            if curr_state == STATE_2:
                self.dir = CLOCKWISE
            elif curr_state == STATE_0:
                self.dir = COUNTERCLOCKWISE
        elif self.curr_state == STATE_2:
            if curr_state == STATE_3:
                self.dir = CLOCKWISE
            elif curr_state == STATE_1:
                self.dir = COUNTERCLOCKWISE
        else:
            if curr_state == STATE_0:
                self.dir = CLOCKWISE
            elif curr_state == STATE_2:
                self.dir = COUNTERCLOCKWISE
        
        if self.curr_state != curr_state:
            self.count += 1
            self.curr_state = curr_state
    
    def start(self):
        """ Start listening to GPIO pins to trigger callback functions """

        def update_state(channel):
            """ update rotation state and call user defined callback functions after complete rotation """
            self.rot_state()
            if self.count // self.counts_per_rotation != 0:
                
                if self.dir == CLOCKWISE:
                    self.on_clockwise_rotate()
                elif self.dir == COUNTERCLOCKWISE:
                    self.on_counterclockwise_rotate()
                
                self.count = 0

        def button_pressed(channel):
            """ Call the user defined callback when button is pressed """
            if not GPIO.input(self.BUTTON_PIN):
                self.on_click()

        self.init()

        # add callback to be called when rotating encoder
        GPIO.add_event_detect(self.A_PIN, GPIO.BOTH, callback=update_state)
        GPIO.add_event_detect(self.B_PIN, GPIO.BOTH, callback=update_state)

        # add callback to be called when button is pressed
        GPIO.add_event_detect(
            self.BUTTON_PIN, GPIO.FALLING, 
            callback=button_pressed, bouncetime=150)

    def stop(self):
        """ clean up the GPIO pins that we were using for this class """
        GPIO.cleanup()

