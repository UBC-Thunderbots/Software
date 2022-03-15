from screen import Screen

ROBOT_ID = 0
CHANNEL_ID = 1

PADDING = 6
BASE_Y = 20
BASE_X = 0
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
        self.ids = [
            redis_dict["robot id"],
            redis_dict["channel id"],
        ]
        self.battery_voltage = redis_dict["battery voltage"]
        self.cap_voltage = redis_dict["cap voltage"]
        self.packet_loss = redis_dict["packet loss"]

        def menu():
            """ Action to go to Menu Screen """
            return {self.screen_actions.CHANGE_SCREEN: "Menu"}

        def robot_id():
            """ Set Robot ID """
            return {
                self.screen_actions.EDIT_SCREEN: {
                    "param": self.ids,
                    "setting": ROBOT_ID,
                    "delta": 1,
                    "redis key": "robot id",
                }
            }

        def channel_id():
            """ Set Channel ID """
            return {
                self.screen_actions.EDIT_SCREEN: {
                    "param": self.ids,
                    "setting": CHANNEL_ID,
                    "delta": 1,
                    "redis key": "channel id",
                }
            }

        # Listing actions for Home Screen
        self.actions = ["Robot ID", "Channel ID", "Menu"]
        self.action_map = {
            self.actions[0]: robot_id,
            self.actions[1]: channel_id,
            self.actions[2]: menu,
        }

        def draw_screen():
            """ Home Screen Layout """
            self.lcd_display.prepare()

            cursor_size = self.font.getsize(">")[0]

            x = cursor_size
            y = 20

            robot_id_str = "Robot ID"
            self.lcd_display.draw.text(
                (x, y), robot_id_str, font=self.font, fill="#FFFFFF"
            )

            x += self.font.getsize(robot_id_str)[0] + cursor_size + PADDING
            channel_id_str = "Channel ID"
            self.lcd_display.draw.text(
                (x, y), channel_id_str, font=self.font, fill="#FFFFFF"
            )

            y += self.font.getsize(channel_id_str)[1]
            x += self.font.getsize(channel_id_str)[0] / 2
            self.lcd_display.draw.text(
                (x, y),
                str(int(self.ids[CHANNEL_ID])),
                font=self.big_font,
                fill="#00ffff",
            )

            x = self.font.getsize(robot_id_str)[0] / 2
            self.lcd_display.draw.text(
                (x, y), str(int(self.ids[ROBOT_ID])), font=self.big_font, fill="#00ffff"
            )

            battery_str = "Battery Voltage: "
            x = cursor_size
            y = BATTERY_VOLTAGE_BASE
            self.lcd_display.draw.text(
                (x, y), battery_str, font=self.font, fill="#FFFFFF"
            )
            x += (self.font.getsize(battery_str))[0]
            self.lcd_display.draw.text(
                (x, 48 + 12),
                str(round(self.battery_voltage, 1)),
                font=self.font,
                fill="#00ffff",
            )

            cap_str = "Capacitor Voltage: "
            x = cursor_size
            y += self.font_size
            self.lcd_display.draw.text((x, y), cap_str, font=self.font, fill="#FFFFFF")
            x += (self.font.getsize(cap_str))[0]
            self.lcd_display.draw.text(
                (x, y), str(round(self.cap_voltage, 1)), font=self.font, fill="#00ffff"
            )

            packet_loss_str = "Packet Loss %: "
            x = cursor_size
            y += self.font_size
            self.lcd_display.draw.text(
                (x, y), packet_loss_str, font=self.font, fill="#FFFFFF"
            )
            x += (self.font.getsize(packet_loss_str))[0]
            self.lcd_display.draw.text(
                (x, y), str(round(self.packet_loss, 1)), font=self.font, fill="#00ffff"
            )

            self.lcd_display.draw.text(
                (cursor_size, self.lcd_display.height - PADDING - self.font_size),
                "Go to Menu screen",
                font=self.font,
                fill="#FFFFFF",
            )

            x = BASE_X
            y = BASE_Y
            if self.curr_action == 1:
                x = self.font.getsize(robot_id_str)[0] + cursor_size + PADDING
            elif self.curr_action == 2:
                y = self.lcd_display.height - PADDING - self.font_size

            self.lcd_display.draw.text((x, y), ">", font=self.font, fill="#ffffff")

        # Pass Home Screen parameters to super class
        super().__init__(
            lcd_display, screen_actions, self.actions, self.action_map, draw_screen
        )

    def update_values(self, redis_dict):
        """ Sync values with those from redis """
        if not self.edit_mode:
            self.ids = [
                redis_dict["robot id"],
                redis_dict["channel id"],
            ]
        self.battery_voltage = redis_dict["battery voltage"]
        self.cap_voltage = redis_dict["cap voltage"]
        self.packet_loss = redis_dict["packet loss"]


# For testing
if __name__ == "__main__":
    import sys

    sys.path.append("../")
    from lcd_user_interface import LcdDisplay

    home_screen = HomeScreen(LcdDisplay())
    home_screen.on_clockwise_rotate()
