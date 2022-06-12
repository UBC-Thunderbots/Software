import time, redis
import digitalio
import board
from PIL import Image, ImageDraw, ImageOps
import adafruit_rgb_display.st7735 as st7735

from software.jetson_nano.display.lcd_user_interface.lcd_user_interface import (
    LcdDisplay,
)

import argparse
from threading import Thread
from software.jetson_nano.display.rotary_encoder.rotary_encoder import RotaryEncoder
from software.jetson_nano.display.screens.home_screen import HomeScreen
from software.jetson_nano.display.screens.menu_screen import MenuScreen
from software.jetson_nano.display.screens.wheels_screen import WheelsScreen
from software.jetson_nano.display.screens.chip_and_kick_screen import ChipAndKickScreen
import software.jetson_nano.display.constants as constants

# Pins for Rotary Encoder
BUTTON_PIN = constants.BUTTON_PIN
PIN_1 = constants.PIN_1
PIN_2 = constants.PIN_2

# These keys indicate how to handle return values
class ScreenActions:
    NONE = 0
    CHANGE_SCREEN = 1
    EDIT_SCREEN = 2
    UPDATE_REDIS = 3


screen_actions = ScreenActions()

# These are the keys for the redis dicationary
redis_keys = [
    "robot id",
    "channel id",
    "battery voltage",
    "cap voltage",
    "packet loss",
    "chip enable",
    "kick enable",
    "chip speed",
    "kick speed",
    "wheels enable",
    "fl wheel speed",
    "fr wheel speed",
    "bl wheel speed",
    "br wheel speed",
]


class RobotUi:
    """
    RobotUI is our user interface for our robots that allows us to quickly get robot 
    diagnostics and change settings of the robot.

    RobotUI uses LcdDisplay to show information, RotaryEncoder to toggle and change different settings,
    and Screen classes to organize the layout of the user interface. 

    RobotUI can synchronize the user interface with a redis server by calling the `poll_redis` 
    function which will continuously update the values with what is in redis every 3 seconds.

    Ensure that the 'spidev' kernel module is loaded while using RobotUI. This can be loaded by 
    the following command: 'sudo modprobe spidev'
    """

    def __init__(self, boot_screen_path):
        """Initialize the RoboUi

        :param boot_screen_path: The path to the tbots logo

        """
        # Initialize redis server and our redis dictionary
        self.redis_client = redis.Redis(
            host="localhost", port=constants.REDIS_PORT_NUMBER, db=0
        )
        self.redis_dict = {}
        for key in redis_keys:
            self.redis_dict[key] = float(self.redis_client.get(key).decode("UTF-8"))
        self.shutdown = False  # This flag will be used to stop polling redis

        # Draw Tbots logo on first boot
        self.lcd_display = LcdDisplay()
        self.lcd_display.draw_image(boot_screen_path)
        self.curr_screen = "Home"

        # All of our screens
        self.screens = {
            "Home": HomeScreen(self.lcd_display, self.redis_dict, screen_actions),
            "Menu": MenuScreen(self.lcd_display, screen_actions),
            "Wheels": WheelsScreen(self.lcd_display, self.redis_dict, screen_actions),
            "Chip and Kick": ChipAndKickScreen(
                self.lcd_display, self.redis_dict, screen_actions
            ),
        }

        def on_click():
            """ Execute on click callback of curr screen """
            action = self.screens[self.curr_screen].on_click()

            if screen_actions.CHANGE_SCREEN == action["screen action"]:
                self.curr_screen = action["value"]
                self.screens[self.curr_screen].update_screen()
                self.lcd_display.show()
            elif screen_actions.UPDATE_REDIS == action["screen action"]:
                self.redis_client.set(action["redis key"], action["value"])
                self.redis_dict[action["redis key"]] = action["value"]
                print(
                    "Key: {}, Value: {}".format(
                        action["redis key"],
                        self.redis_client.get(action["redis key"]).decode("UTF-8"),
                    )
                )

        def on_clockwise_rotate():
            """ Execute the clockwise rotate callback of curr screen """
            self.screens[self.curr_screen].on_clockwise_rotate()

        def on_counterclockwise_rotate():
            """ Execute the counterclockwise rotate callback of curr screen """
            self.screens[self.curr_screen].on_counterclockwise_rotate()

        self.rotary_encoder = RotaryEncoder(
            PIN_1,
            PIN_2,
            BUTTON_PIN,
            on_clockwise_rotate,
            on_counterclockwise_rotate,
            on_click,
        )

        self.rotary_encoder.start()

    def poll_redis(self, timeout=3):
        """ Update redis dict every timeout seconds """
        while not self.shutdown:
            for key in redis_keys:
                self.redis_dict[key] = float(self.redis_client.get(key).decode("UTF-8"))

            for screen_name, screen in self.screens.items():
                if screen_name != "Menu":
                    screen.update_values(self.redis_dict)
            time.sleep(timeout)

    def stop(self):
        """ Cleanup the GPIO pins """
        self.shutdown = True
        self.rotary_encoder.stop()


if __name__ == "__main__":

    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--path_to_boot_screen",
        required=True,
        type=str,
        help="path to image to show on boot",
    )
    args = vars(ap.parse_args())

    def init_redis():
        redis_client = redis.Redis(
            host="localhost", port=constants.REDIS_PORT_NUMBER, db=0
        )
        for key in redis_keys:
            redis_client.set(key, 0)

    def start_polling(robot_ui):
        robot_ui.poll_redis()

    robot_ui = RobotUi(boot_screen_path=args["path_to_boot_screen"])
    thread = Thread(target=start_polling, args=(robot_ui,))
    thread.start()

    while True:
        time.sleep(1)
