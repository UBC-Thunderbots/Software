import time, redis
import digitalio
import board
from PIL import Image, ImageDraw, ImageOps
import adafruit_rgb_display.st7735 as st7735

# TODO: test if bazel build works
# import sys
# sys.path.append("./rotary_encoder/")
# sys.path.append("./lcd_user_interface/")
# sys.path.append("./screens/")
# from rotary_encoder import RotaryEncoder
# from lcd_user_interface import LcdDisplay
# from home_screen import HomeScreen
# from menu_screen import MenuScreen
# from wheels_screen import WheelsScreen
# from chip_and_kick_screen import ChipAndKickScreen

from software.display.lcd_user_interface import LcdDisplay
from software.display.rotary_encoder import RotaryEncoder
from software.display.screens.home_scren import HomeScreen
from software.display.screens.menu_scren import MenuScreen
from software.display.screens.wheels_scren import WheelsScreen
from software.display.screens.chip_and_kick_scren import ChipAndKickScreen
import constants

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
    This is the top level class for the robot user interface. It ties together the 
    rotary encoder, lcd display, redis server, and all the screens.
    """

    def __init__(self):

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
        self.lcd_display.draw_image("./lcd_user_interface/imgs/tbots.jpg")
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


# For testing
if __name__ == "__main__":
    import subprocess
    from threading import Thread

    def init_redis():
        redis_client = redis.Redis(
            host="localhost", port=constants.REDIS_PORT_NUMBER, db=0
        )
        redis_dict = {}
        for key in redis_keys:
            redis_client.set(key, 0)

    def start_polling(robot_ui):
        robot_ui.poll_redis()

    # start redis server
    cmd = "cd ../linux_configs/redis_config && sudo docker-compose up -d"
    st, out = subprocess.getstatusoutput(cmd)
    init_redis()

    robot_ui = RobotUi()
    thread = Thread(target=start_polling, args=(robot_ui,))
    thread.start()

    print("Press any key to exit...")
    input()
    robot_ui.stop()
    thread.join()

    # stop redis server
    cmd = "cd ../linux_configs/redis_config && sudo docker-compose down"
    st, out = subprocess.getstatusoutput(cmd)
