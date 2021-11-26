import Jetson.GPIO as GPIO  # pip install Jetson.GPIO

"""
The naming convention for the pins used to determine rotation state follow the labels
shown on page 4 of our rotary encoder's datasheet: https://www.bourns.com/docs/Product-Datasheets/PEC12R.pdf

The state of a rotary encoder is dependent on the values read on pin 1 and pin 2.
The input read on pin 1 and pin 2 can either be of value 0 or 1 and therefore we have four
possible states. The encoder will transition through these four states during a 
single rotation and we can use these state transitions to determine whether the encoder 
has been rotated clockwise or counterclockwise.

Pins 1 and 2 go through 4 states for a single rotation.
The following are values for clockwise rotation:
   Pin 1    Pin 2
     0        0
     0        1
     1        1
     1        0

The list below defines the states that our rotary encoder may have. The first index in a
tuple is the value read from pin 1 and the second index is the value read from pin 2
"""
STATES = [(0, 0), (0, 1), (1, 1), (1, 0)]

BOUNCETIME = 150

# curr_state : next_state  indicates clockwise rotation
clockwise_state_transitions = {
    STATES[0]: STATES[1],
    STATES[1]: STATES[2],
    STATES[2]: STATES[3],
    STATES[3]: STATES[0],
}

# curr_state : next_state  indicates counter clockwise rotation
counterclockwise_state_transitions = {
    STATES[0]: STATES[3],
    STATES[1]: STATES[0],
    STATES[2]: STATES[1],
    STATES[3]: STATES[2],
}


class RotaryEncoder:
    def __init__(
        self,
        PIN_1,
        PIN_2,
        BUTTON_PIN,
        on_clockwise_rotate,
        on_counterclockwise_rotate,
        on_click,
    ):
        """
        @param PIN_1, the Jetson Nano board pin number used for channel 1 input of rotary encoder; pin requires pull up resistor
        @param PIN_2, the Jetson Nano board pin number used for channel 2 input of rotary encoder; pin requires pull up resistor
        @param BUTTON_PIN, the Jetson Nano board pin number used for push button input of rotary encoder; pin requires pull up resistor
        @param on_clockwise_rotate, callback function to be called when rotating clockwise, callback takes no params
        @param on_counterclockwise_rotate, callback function to be called when rotatin counterclockwise, callback takes no params
        @param on_click, callback function to be called when push button is clicked, callback takes no params
        """
        self.PIN_1 = PIN_1
        self.PIN_2 = PIN_2
        self.BUTTON_PIN = BUTTON_PIN

        self.on_clockwise_rotate = on_clockwise_rotate
        self.on_counterclockwise_rotate = on_counterclockwise_rotate
        self.on_click = on_click

        self.transitions_per_rotation = len(STATES)

    def setup(self):
        """ Initialize GPIO pins and rotary encoder state """
        GPIO.setmode(GPIO.BOARD)

        # These pins will need external pullups set up
        GPIO.setup(self.PIN_1, GPIO.IN)
        GPIO.setup(self.PIN_2, GPIO.IN)
        GPIO.setup(self.BUTTON_PIN, GPIO.IN)

        pin_1_state = GPIO.input(self.PIN_1)
        pin_2_state = GPIO.input(self.PIN_2)

        self.curr_state = (pin_1_state, pin_2_state)

        self.count = 0

    def rot_state(self):
        """ Update the current state and count state transitions """
        pin_1_state = GPIO.input(self.PIN_1)
        pin_2_state = GPIO.input(self.PIN_2)
        next_state = (pin_1_state, pin_2_state)
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
            if self.count // self.transitions_per_rotation != 0:

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
        GPIO.add_event_detect(self.PIN_1, GPIO.BOTH, callback=on_rotation)
        GPIO.add_event_detect(self.PIN_2, GPIO.BOTH, callback=on_rotation)

        # add callback to be called when button is pressed
        GPIO.add_event_detect(
            self.BUTTON_PIN,
            GPIO.FALLING,
            callback=button_pressed,
            bouncetime=BOUNCETIME,
        )

    def stop(self):
        """ clean up the GPIO pins that we were using for this class """
        GPIO.cleanup()
