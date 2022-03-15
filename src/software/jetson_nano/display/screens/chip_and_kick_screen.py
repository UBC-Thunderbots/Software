from screen import Screen

CHIP = 0
KICK = 1

PADDING = 6
BASE_Y = 20


class ChipAndKickScreen(Screen):
    """
    This screen is used to edit the chip and kick speed settings
    """
    def __init__(self, lcd_display, redis_dict, screen_actions):
        """
        @param lcd_display, an instance of the LcdDisplay class
        @param redis_dict, a dict of values from redis client to init variables on this screen
        @param screen_actions, an instance of ScreenActions class
        """
        self.enable = False if redis_dict["chip and kick enable"] == 0 else True
        self.speeds = [
            redis_dict["chip speed"],
            redis_dict["kick speed"],
        ]

        # Defining this screens actions
        def menu():
            """ Go to the menu screen """
            self.curr_action = 0

            return {self.screen_actions.CHANGE_SCREEN: "Menu"}

        def set_chip_and_kick_speed():
            """ Enable and disable settings """
            if self.enable:
                self.enable = False
            else:
                self.enable = True

            self.update_screen()
            return {
                self.screen_actions.UPDATE_REDIS: {
                    "redis key": "chip and kick enable",
                    "value": 1 if self.enable else 0,
                }
            }

        def set_chip():
            """ Set chip speed """
            return {
                self.screen_actions.EDIT_SCREEN: {
                    "param": self.speeds,
                    "setting": CHIP,
                    "delta": 0.5,
                    "redis key": "chip speed",
                }
            }

        def set_kick():
            """ Set kick speed """
            return {
                self.screen_actions.EDIT_SCREEN: {
                    "param": self.speeds,
                    "setting": KICK,
                    "delta": 0.5,
                    "redis key": "kick speed",
                }
            }

        # Listing actions for Config Wheels Screen
        self.actions = ["Set Chip and Kick Speed", "Chip", "Kick", "Menu"]
        self.action_map = {
            self.actions[0]: set_chip_and_kick_speed,
            self.actions[1]: set_chip,
            self.actions[2]: set_kick,
            self.actions[3]: menu,
        }

        def draw_screen():
            """ Wheels Screen Layout """
            self.lcd_display.prepare()

            cursor = ">"
            cursor_size = self.font.getsize(cursor)[0]
            cursor_pos_x = 0
            if self.curr_action != len(self.actions) - 1:
                cursor_pos_y = 20 + self.font_size * self.curr_action
            else:
                cursor_pos_y = self.lcd_display.height - self.font_size - PADDING

            self.lcd_display.draw.text(
                (cursor_pos_x, cursor_pos_y), cursor, font=self.font, fill="#ffffff"
            )

            # x and y coordinates for drawing on screen
            x = cursor_size
            y = BASE_Y

            set_chip_and_kick_str = "Set Chip & Kick: "
            self.lcd_display.draw.text(
                (x, y), set_chip_and_kick_str, font=self.font, fill="#ffffff"
            )
            x += self.font.getsize(set_chip_and_kick_str)[0]
            self.lcd_display.draw.text(
                (x, y),
                "{}".format(self.enable),
                font=self.font,
                fill="#00ff00" if self.enable else "#0000ff",
            )

            x = cursor_size
            y += self.font_size
            chip_str = "Chip Speed: "
            self.lcd_display.draw.text((x, y), chip_str, font=self.font, fill="#ffffff")
            x += (self.font.getsize(chip_str))[0]
            self.lcd_display.draw.text(
                (x, y), str(round(self.speeds[CHIP], 1)), font=self.font, fill="#00ffff"
            )

            x = cursor_size
            y += self.font_size
            kick_str = "Kick Speed: "
            self.lcd_display.draw.text((x, y), kick_str, font=self.font, fill="#ffffff")
            x += (self.font.getsize(kick_str))[0]
            self.lcd_display.draw.text(
                (x, y), str(round(self.speeds[KICK], 1)), font=self.font, fill="#00ffff"
            )

            x = cursor_size
            y = self.lcd_display.height - self.font_size - PADDING
            self.lcd_display.draw.text(
                (x, y), "Go to Menu screen", font=self.font, fill="#ffffff",
            )

        # Pass Wheel Screen parameters to super class
        super().__init__(
            lcd_display, screen_actions, self.actions, self.action_map, draw_screen
        )

    def update_values(self, redis_dict):
        if not self.edit_mode:
            self.enable = False if redis_dict["chip and kick enable"] == 0 else True
            self.speeds = [
                redis_dict["chip speed"],
                redis_dict["kick speed"],
            ]
