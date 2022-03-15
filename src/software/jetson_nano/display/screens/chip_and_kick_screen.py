from screen import Screen

ENABLE_INDEX = 0
CHIP_INDEX = 1
KICK_INDEX = 2

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
        actions = [              
            {
                "redis key": "chip and kick enable",
                "value": redis_dict["chip and kick enable"],
                "type": bool,
                "delta": None,
                "screen action": screen_actions.UPDATE_REDIS
            },
            {
                "redis key": "chip speed",
                "value": redis_dict["chip speed"],
                "type": float,
                "delta": 0.5,
                "screen action": screen_actions.EDIT_SCREEN
            },
            {
                "redis key": "kick speed",
                "value": redis_dict["kick speed"],
                "type": float,
                "delta": 0.5,
                "screen action": screen_actions.EDIT_SCREEN
            },
            {
                "redis key": None,
                "value": "Menu",
                "type": str,
                "delta": None,
                "screen action": screen_actions.CHANGE_SCREEN,
            },
        ]

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
            en = True if self.actions[ENABLE_INDEX]["value"] else False

            set_chip_and_kick_str = "Set Chip & Kick: "
            self.lcd_display.draw.text(
                (x, y), set_chip_and_kick_str, font=self.font, fill="#ffffff"
            )
            x += self.font.getsize(set_chip_and_kick_str)[0]
            self.lcd_display.draw.text(
                (x, y),
                "{}".format(en),
                font=self.font,
                fill="#00ff00" if en else "#0000ff",
            )

            x = cursor_size
            y += self.font_size
            chip_str = "Chip Speed: "
            self.lcd_display.draw.text((x, y), chip_str, font=self.font, fill="#ffffff")
            x += (self.font.getsize(chip_str))[0]
            self.lcd_display.draw.text(
                (x, y), str(round(self.actions[CHIP_INDEX]["value"], 1)), font=self.font, fill="#00ffff"
            )

            x = cursor_size
            y += self.font_size
            kick_str = "Kick Speed: "
            self.lcd_display.draw.text((x, y), kick_str, font=self.font, fill="#ffffff")
            x += (self.font.getsize(kick_str))[0]
            self.lcd_display.draw.text(
                (x, y), str(round(self.actions[KICK_INDEX]["value"], 1)), font=self.font, fill="#00ffff"
            )

            x = cursor_size
            y = self.lcd_display.height - self.font_size - PADDING
            self.lcd_display.draw.text(
                (x, y), "Go to Menu screen", font=self.font, fill="#ffffff",
            )

        # Pass Wheel Screen parameters to super class
        super().__init__(lcd_display, screen_actions, draw_screen, actions)

    def update_values(self, redis_dict):
        if not self.edit_mode:
            self.actions[ENABLE_INDEX]["value"] = 1 if redis_dict["chip and kick enable"] else 0
            self.actions[CHIP_INDEX]["value"] = redis_dict["chip speed"]
            self.actions[KICK_INDEX]["value"] = redis_dict["kick speed"]
