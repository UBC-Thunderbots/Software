import Jetson.GPIO as GPIO

"""
The state of a rotary encoder is dependent on the values read on pin A and pin B.
The input read on pin A and pin B can either be 0 or 1 and therefore we have four
possible states. The encoder will transition through these four states during a 
single rotation and we can use these state transitions to determine whether the encoder 
has been rotated clockwise or counterclockwise.

Channel B and A go through 4 states for a single rotation.
The following are values for clockwise rotation:
  Channel B Channel A
     0          0
     0          1
     1          1
     1          0

The defined states below are tuples where the first index is the value read from pin B and
the second index the value read from pin A.
"""
STATE_0 = (0, 0)
STATE_1 = (0, 1)
STATE_2 = (1, 1)
STATE_3 = (1, 0)

NUM_STATES = 4

CLOCKWISE = 1
COUNTERCLOCKWISE = -1
IDLE = 0

# curr_state : next_state  indicates clockwise rotation
clockwise_state_transitions =\
{
    STATE_0: STATE_1,
    STATE_1: STATE_2,
    STATE_2: STATE_3,
    STATE_3: STATE_0
}

# curr_state : next_state  indicates counter clockwise rotation
counterclockwise_state_transitions =\
{
    STATE_0: STATE_3,
    STATE_1: STATE_0,
    STATE_2: STATE_1,
    STATE_3: STATE_2
}

"""
@param A_PIN, the Jetson Nano board pin number used for channel A input of rotary encoder; pin requires pull up resistor
@param B_PIN, the Jetson Nano board pin number used for channel B input of rotary encoder; pin requires pull up resistor
@param BUTTON_PIN, the Jetson Nano board pin number used for push button input of rotary encoder; pin requires pull up resistor
@param on_clockwise_rotate, callback function to be called when rotating clockwise, callback takes no params
@param on_counterclockwise_rotate, callback function to be called when rotatin counterclockwise, callback takes no params
@param on_click, callback function to be called when push button is clicked, callback takes no params
"""
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

        self.counts_per_rotation = NUM_STATES

    def setup(self):
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

    def rot_state(self):
        """ Update the current state and count state transitions """
        a_state = GPIO.input(self.A_PIN)
        b_state = GPIO.input(self.B_PIN)
        next_state = (b_state, a_state)
        prev_state = self.curr_state

        if self.curr_state != next_state:
            self.count += 1
            self.curr_state = next_state
            
        return prev_state

    def start(self):
        """ Start listening to GPIO pins to trigger callback functions """
        
        def on_rotation(channel):
            """ Update rotation state and call user defined callback functions after complete rotation """
            prev_state = self.rot_state()
            if self.count // self.counts_per_rotation != 0:

                if clockwise_state_transitions[prev_state] == self.curr_state:
                    self.on_clockwise_rotate()
                elif counterclockwise_state_transitions[prev_state] == self.curr_state:
                    self.on_counterclockwise_rotate() 

                self.count = 0

        def button_pressed(channel):
            """ Call the user defined callback when button is pressed """
            if not GPIO.input(self.BUTTON_PIN):
                self.on_click()

        self.setup()

        # add callback to be called when rotating encoder
        GPIO.add_event_detect(self.A_PIN, GPIO.BOTH, callback=on_rotation)
        GPIO.add_event_detect(self.B_PIN, GPIO.BOTH, callback=on_rotation)

        # add callback to be called when button is pressed
        GPIO.add_event_detect(
            self.BUTTON_PIN, GPIO.FALLING,
            callback=button_pressed, bouncetime=150)

    def stop(self):
        """ clean up the GPIO pins that we were using for this class """
        GPIO.cleanup()
