import sys
sys.path.append('./screens/')
import digitalio
import board
from PIL import Image, ImageDraw, ImageOps
import adafruit_rgb_display.st7735 as st7735

from lcd_user_interface import LcdDisplay
from rotary_encoder import RotaryEncoder
from home_screen import HomeScreen
from menu_screen import MenuScreen
from wheels_screen import WheelsScreen
from chip_and_kick_screen import ChipAndKickScreen

# Pins for Rotary Encoder
BUTTON_PIN = 'DAP4_DOUT'    # BOARD 35, TEGRA_SOC: 'DAP4_FS'
PIN_1 = 'GPIO_PE6'          # BOARD 40, TEGRA_SOC: 'DAP4_DOUT' 
PIN_2 = 'DAP4_FS'           # BOARD 33, TEGRA_SOC: 'GPIO_PE6'

# These keys indicate how to handle return values
key_list = {
    "none": 0,
    "change screen": 1,
    "edit": 2
}

class RobotUi:
    def __init__(self):
        self.lcd_display = LcdDisplay()
        self.lcd_display.draw_image('./imgs/tbots.jpg')
        self.curr_screen = "Home"

        # All of our screens
        self.screens = {
            "Home": HomeScreen(self.lcd_display, key_list),
            "Menu": MenuScreen(self.lcd_display, key_list),
            "Wheels": WheelsScreen(self.lcd_display, key_list),
            "Chip and Kick": ChipAndKickScreen(self.lcd_display, key_list)
        }

        def on_click():
            """ Execute on click callback of curr screen """
            action = self.screens[self.curr_screen].on_click()
            if key_list["change screen"] in action: 
                self.curr_screen = action[key_list["change screen"]]
                self.screens[self.curr_screen].draw_screen()
                self.lcd_display.show()

        def on_clockwise_rotate():
            """ Execute the clockwise rotate callback of curr screen """
            self.screens[self.curr_screen].on_clockwise_rotate()
            
        def on_counterclockwise_rotate():
            """ Execute the counterclockwise rotate callback of curr screen """
            self.screens[self.curr_screen].on_counterclockwise_rotate()

        self.rotary_encoder = \
            RotaryEncoder(
                PIN_1, 
                PIN_2, 
                BUTTON_PIN,
                on_clockwise_rotate,
                on_counterclockwise_rotate, 
                on_click
            )

        self.rotary_encoder.start()

    def stop(self):
        self.rotary_encoder.stop()

# For testing
if __name__ == '__main__':
    robot_ui = RobotUi()   
    print('Press any key to exit...')
    input()
    robot_ui.stop()
