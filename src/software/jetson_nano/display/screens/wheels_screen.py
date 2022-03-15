from screen import Screen

ENABLE_INDEX = 0
FL_INDEX = 1
FR_INDEX = 2
BL_INDEX = 3
BR_INDEX = 4

PADDING = 6
BASE_Y = 20


class WheelsScreen(Screen):
    """
    This screen is used to edit wheel speed settings
    """
    def __init__(self, lcd_display, redis_dict, screen_actions):
        """
        @param lcd_display, an instance of the LcdDisplay class
        @param redis_dict, a dict of values from redis client to init variables on this screen
        @param screen_actions, an instance of ScreenActions class
        """
        actions = [              
            {
                "redis key": "wheels enable",
                "value": redis_dict["wheels enable"],
                "type": bool,
                "delta": None,
                "screen action": screen_actions.UPDATE_REDIS
            },
            {
                "redis key": "fl wheel speed",
                "value": redis_dict["fl wheel speed"],
                "type": float,
                "delta": 0.5,
                "screen action": screen_actions.EDIT_SCREEN
            },
            {
                "redis key": "fr wheel speed",
                "value": redis_dict["fr wheel speed"],
                "type": float,
                "delta": 0.5,
                "screen action": screen_actions.EDIT_SCREEN
            },
            {
                "redis key": "bl wheel speed",
                "value": redis_dict["bl wheel speed"],
                "type": float,
                "delta": 0.5,
                "screen action": screen_actions.EDIT_SCREEN
            },
            {
                "redis key": "br wheel speed",
                "value": redis_dict["br wheel speed"],
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

            # Displaying the cursor
            cursor = ">"
            cursor_size = self.font.getsize(cursor)[0]
            cursor_pos_x = 0
            if self.curr_action != len(self.actions) - 1:
                cursor_pos_y = BASE_Y + self.font_size * self.curr_action
            else:
                cursor_pos_y = self.lcd_display.height - self.font_size - PADDING

            self.lcd_display.draw.text(
                (cursor_pos_x, cursor_pos_y), cursor, font=self.font, fill="#ffffff"
            )

            # x and y coordinates for drawing on screen
            x = cursor_size
            y = BASE_Y
            en = True if self.actions[ENABLE_INDEX]["value"] else False

            set_wheel_speed_str = "Set Wheel Speed: "
            self.lcd_display.draw.text(
                (x, y), set_wheel_speed_str, font=self.font, fill="#ffffff"
            )
            x += self.font.getsize(set_wheel_speed_str)[0]
            self.lcd_display.draw.text(
                (x, y),
                "{}".format(en),
                font=self.font,
                fill="#00ff00" if en else "#0000ff",
            )

            x = cursor_size
            y += self.font_size
            front_left_str = "Front Left: "
            self.lcd_display.draw.text(
                (x, y), front_left_str, font=self.font, fill="#ffffff"
            )
            x += self.font.getsize(front_left_str)[0]
            self.lcd_display.draw.text(
                (x, y),
                str(round(self.actions[FL_INDEX]["value"], 1)),
                font=self.font,
                fill="#00ffff",
            )

            x = cursor_size
            y += self.font_size
            front_right_str = "Front Right: "
            self.lcd_display.draw.text(
                (x, y), front_right_str, font=self.font, fill="#ffffff"
            )
            x += self.font.getsize(front_right_str)[0]
            self.lcd_display.draw.text(
                (x, y),
                str(round(self.actions[FR_INDEX]["value"], 1)),
                font=self.font,
                fill="#00ffff",
            )

            x = cursor_size
            y += self.font_size
            back_left_str = "Back Left: "
            self.lcd_display.draw.text(
                (x, y), back_left_str, font=self.font, fill="#ffffff"
            )
            x += self.font.getsize(back_left_str)[0]
            self.lcd_display.draw.text(
                (x, y),
                str(round(self.actions[BL_INDEX]["value"], 1)),
                font=self.font,
                fill="#00ffff",
            )

            x = cursor_size
            y += self.font_size
            back_right_str = "Back Right: "
            self.lcd_display.draw.text(
                (x, y), back_right_str, font=self.font, fill="#ffffff"
            )
            x += self.font.getsize(back_right_str)[0]
            self.lcd_display.draw.text(
                (x, y),
                str(round(self.actions[BR_INDEX]["value"], 1)),
                font=self.font,
                fill="#00ffff",
            )

            x = cursor_size
            y = self.lcd_display.height - self.font_size - PADDING
            self.lcd_display.draw.text(
                (x, y), "Go to Menu screen", font=self.font, fill="#ffffff",
            )

        # Pass Wheel Screen parameters to super class
        super().__init__(
            lcd_display, screen_actions, draw_screen, actions
        )

    def update_values(self, redis_dict):
        """ Sync values with those from redis """
        if not self.edit_mode:
            self.actions[ENABLE_INDEX]["value"] = 1 if redis_dict["wheels enable"] else 0
            self.actions[FL_INDEX]["value"] = redis_dict["fl wheel speed"]
            self.actions[FR_INDEX]["value"] = redis_dict["fr wheel speed"]
            self.actions[BL_INDEX]["value"] = redis_dict["bl wheel speed"]
            self.actions[BR_INDEX]["value"] = redis_dict["br wheel speed"]
