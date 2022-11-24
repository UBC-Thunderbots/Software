from software.jetson_nano.display.screens.screen import Screen
import software.jetson_nano.display.constants as constants

ROBOT_ID_INDEX = 0
CHANNEL_ID_INDEX = 1


BATTERY_VOLTAGE_BASE = 60


class HomeScreen(Screen):
    """
    This is the dashboard screen which shows basic robot diagnostics
    """

    def __init__(self, lcd_display, redis_dict, screen_actions):
        """
        @param lcd_display, an instance of the LcdDisplay class
        @param redis_dict, a dict of values from redis client to init variables on this screen
        @param screen_actions, an instance of ScreenActions class
        """
        actions = [
            {
                "redis key": ROBOT_ID_REDIS_KEY,
                "value": redis_dict[ROBOT_ID_REDIS_KEY],
                "type": float,
                "delta": 1,
                "screen action": screen_actions.EDIT_SCREEN,
            },
            {
                "redis key": ROBOT_MULTICAST_CHANNEL_REDIS_KEY,
                "value": redis_dict[ROBOT_MULTICAST_CHANNEL_REDIS_KEY],
                "type": float,
                "delta": 1,
                "screen action": screen_actions.EDIT_SCREEN,
            },
            {
                "redis key": None,
                "value": "Menu",
                "type": str,
                "delta": None,
                "screen action": screen_actions.CHANGE_SCREEN,
            },
        ]
        self.battery_voltage = redis_dict["/battery_voltage"]
        self.cap_voltage = redis_dict["/cap_voltage"]
        self.current_draw = redis_dict["/current_draw"]

        def draw_screen():
            """ Home Screen Layout """

            def setup_robot_id(x, y):
                """ Draw the robot id setting on the screen """
                robot_id_str = "Robot ID"
                self.lcd_display.draw.text(
                    (x, y), robot_id_str, font=self.font, fill=constants.WHITE
                )

                channel_id_x = (
                    x
                    + self.font.getsize(robot_id_str)[0]
                    + cursor_size
                    + constants.PADDING
                )

                x = self.font.getsize(robot_id_str)[0] / 2
                y += self.font_size
                self.lcd_display.draw.text(
                    (x, y),
                    str(int(self.actions[ROBOT_ID_INDEX]["value"])),
                    font=self.big_font,
                    fill=constants.YELLOW,
                )

                return channel_id_x

            def setup_channel_id(x, y):
                """ Draw the channel id setting on the screen """
                channel_id_str = "Channel ID"
                self.lcd_display.draw.text(
                    (x, y), channel_id_str, font=self.font, fill=constants.WHITE
                )
                y += self.font.getsize(channel_id_str)[1]
                x += self.font.getsize(channel_id_str)[0] / 2
                self.lcd_display.draw.text(
                    (x, y),
                    str(int(self.actions[CHANNEL_ID_INDEX]["value"])),
                    font=self.big_font,
                    fill=constants.YELLOW,
                )

            def setup_battery_voltage(x, y):
                """ Draw the battery voltage diagnostic on the screen """
                battery_str = "Battery Voltage: "
                x = cursor_size

                y = BATTERY_VOLTAGE_BASE
                self.lcd_display.draw.text(
                    (x, y), battery_str, font=self.font, fill=constants.WHITE
                )
                x += (self.font.getsize(battery_str))[0]
                self.lcd_display.draw.text(
                    (x, y),
                    str(round(self.battery_voltage, 1)),
                    font=self.font,
                    fill=constants.YELLOW,
                )

                return y + self.font_size

            def setup_capacitor_voltage(x, y):
                """ Draw the capacitor voltage diagnostic on the screen """
                cap_str = "Capacitor Voltage: "
                x = cursor_size
                self.lcd_display.draw.text(
                    (x, y), cap_str, font=self.font, fill=constants.WHITE
                )
                x += (self.font.getsize(cap_str))[0]
                self.lcd_display.draw.text(
                    (x, y),
                    str(round(self.cap_voltage, 1)),
                    font=self.font,
                    fill=constants.YELLOW,
                )

                return y + self.font_size

            def setup_current_draw(x, y):
                """ Draw the current draw on screen """
                current_draw_str = "Current Draw %: "
                x = cursor_size
                self.lcd_display.draw.text(
                    (x, y), current_draw_str, font=self.font, fill=constants.WHITE
                )
                x += (self.font.getsize(current_draw_str))[0]
                self.lcd_display.draw.text(
                    (x, y),
                    str(round(self.current_draw, 1)),
                    font=self.font,
                    fill=constants.YELLOW,
                )

            self.lcd_display.prepare()

            cursor_size = self.font.getsize(constants.CURSOR)[0]

            x = cursor_size
            y = constants.BASE_Y

            x = setup_robot_id(x, y)
            setup_channel_id(x, y)

            y = setup_battery_voltage(x, y)
            y = setup_capacitor_voltage(x, y)
            setup_current_draw(x, y)

            # draw the back screen
            self.lcd_display.draw.text(
                (
                    cursor_size,
                    self.lcd_display.height - constants.PADDING - self.font_size,
                ),
                "Go to Menu screen",
                font=self.font,
                fill=constants.WHITE,
            )

            # draw the cursor
            x = constants.BASE_X
            y = constants.BASE_Y
            if self.curr_action == 1:
                x = self.font.getsize("Robot ID")[0] + cursor_size + constants.PADDING
            elif self.curr_action == 2:
                y = self.lcd_display.height - constants.PADDING - self.font_size

            self.lcd_display.draw.text(
                (x, y), constants.CURSOR, font=self.font, fill=constants.WHITE
            )

        # Pass Home Screen parameters to super class
        super().__init__(lcd_display, screen_actions, actions, draw_screen)

    def update_values(self, redis_dict):
        """ Sync values with those from redis """
        if not self.edit_mode:
            for i in range(self.len):
                if self.actions[i]["redis key"] == None:
                    continue

                self.actions[i]["value"] = redis_dict[self.actions[i]["redis key"]]

        self.battery_voltage = redis_dict["/battery_voltage"]
        self.cap_voltage = redis_dict["/cap_voltage"]
        self.current_draw = redis_dict["/current_draw"]


# For testing
if __name__ == "__main__":
    import sys

    sys.path.append("../")
    from lcd_user_interface import LcdDisplay

    home_screen = HomeScreen(LcdDisplay())
    home_screen.on_clockwise_rotate()
