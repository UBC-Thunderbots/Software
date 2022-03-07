import sys, time, redis
import digitalio
import board
from PIL import Image, ImageDraw, ImageOps
import adafruit_rgb_display.st7735 as st7735

sys.path.append("./screens/")
sys.path.append("./lcd_user_interface/")
sys.path.append("./rotary_encoder/")
from lcd_user_interface import LcdDisplay
from rotary_encoder import RotaryEncoder
from home_screen import HomeScreen
from menu_screen import MenuScreen
from wheels_screen import WheelsScreen
from chip_and_kick_screen import ChipAndKickScreen

# Pins for Rotary Encoder
BUTTON_PIN = "DAP4_DOUT"  # BOARD 35, TEGRA_SOC: 'DAP4_FS'
PIN_1 = "GPIO_PE6"  # BOARD 40, TEGRA_SOC: 'DAP4_DOUT'
PIN_2 = "DAP4_FS"  # BOARD 33, TEGRA_SOC: 'GPIO_PE6'

# These keys indicate how to handle return values
status_codes = {"none": 0, "change screen": 1, "edit": 2, "update redis": 3}

# These are the keys for the redis dicationary
redis_keys = [
    "robot id",
    "channel id",
    "battery voltage",
    "cap voltage",
    "packet loss",
    "chip and kick enable",
    "chip speed",
    "kick speed",
    "wheels enable",
    "fl wheel speed",
    "fr wheel speed",
    "bl wheel speed",
    "br wheel speed",
]

"""
This is the top level class for the robot user interface. It ties together the 
rotary encoder, lcd display, redis server, and all the screens.
"""
class RobotUi:
    def __init__(self):

        # Initialize redis server and our redis dictionary
        self.r = redis.Redis(host="localhost", port=6379, db=0)
        self.redis_dict = {}
        for key in redis_keys:
            self.redis_dict[key] = float(self.r.get(key).decode("UTF-8"))
        self.shutdown = False # This flag will be used to stop polling redis 

        # Draw Tbots logo on first boot
        self.lcd_display = LcdDisplay()
        self.lcd_display.draw_image("./lcd_user_interface/imgs/tbots.jpg")
        self.curr_screen = "Home"

        # All of our screens
        self.screens = {
            "Home": HomeScreen(self.lcd_display, self.redis_dict, status_codes),
            "Menu": MenuScreen(self.lcd_display, status_codes),
            "Wheels": WheelsScreen(self.lcd_display, self.redis_dict, status_codes),
            "Chip and Kick": ChipAndKickScreen(
                self.lcd_display, self.redis_dict, status_codes
            ),
        }

        def on_click():
            """ Execute on click callback of curr screen """
            action = self.screens[self.curr_screen].on_click()

            if status_codes["change screen"] in action:
                self.curr_screen = action[status_codes["change screen"]]
                self.screens[self.curr_screen].draw_screen()
                self.lcd_display.show()
            elif status_codes["update redis"] in action:
                payload = action[status_codes["update redis"]]
                self.r.set(payload["redis key"], payload["value"])
                self.redis_dict[payload["redis key"]] = payload["value"]
                #print("Key: {}, Value: {}".format(payload["redis key"], self.r.get(payload["redis key"]).decode("UTF-8")))

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

    def poll_redis(self):
        """ Update redis dict every 10 seconds """
        while not self.shutdown:
            for key in redis_keys:
                self.redis_dict[key] = float(self.r.get(key).decode("UTF-8"))
            
            for _, screen in self.screens.items():
                if _ != "Menu":
                    screen.update_values(self.redis_dict)
            time.sleep(10)

    def stop(self):
        """ Cleanup the GPIO pins """
        self.shutdown = True
        self.rotary_encoder.stop()


# For testing
if __name__ == "__main__":
    sys.path.append("./redis-test/")
    import subprocess
    from threading import Thread
    
    def init_redis():
        r = redis.Redis(host="localhost", port=6379, db=0)
        redis_dict = {}
        for key in redis_keys:
            r.set(key, 0)

    def start_polling(robot_ui):
        robot_ui.poll_redis()

    # start redis server
    cmd = "cd redis-test && sudo docker-compose up -d"
    st, out = subprocess.getstatusoutput(cmd)
    init_redis()

    robot_ui = RobotUi()
    thread = Thread(target=start_polling, args=(robot_ui, ))
    thread.start()

    print("Press any key to exit...")
    input()
    robot_ui.stop()
    thread.join()

    # stop redis server
    cmd = "cd redis-test && sudo docker-compose down"
    st, out = subprocess.getstatusoutput(cmd)
