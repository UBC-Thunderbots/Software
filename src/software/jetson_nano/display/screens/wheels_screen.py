from screen import Screen

FRONT_LEFT = 0
FRONT_RIGHT = 1
BACK_LEFT = 2
BACK_RIGHT = 3

PADDING = 6
BASE_Y = 20

"""
This screen is used to edit wheel speed settings
"""
class WheelsScreen(Screen):
    def __init__(self, lcd_display, redis_dict, status_codes):
        self.enable = False if redis_dict["wheels enable"] == 0 else True
        self.wheel_speeds = [
            redis_dict["fl wheel speed"],
            redis_dict["fr wheel speed"],
            redis_dict["bl wheel speed"],
            redis_dict["br wheel speed"],
        ]

        # Defining this screens actions
        def menu():
            """ Go to the menu screen """
            self.curr_action = 0
            return {self.status_codes["change screen"]: "Menu"}

        def set_wheel_speed():
            """ Enable and disable settings """
            if self.enable:
                self.enable = False
            else:
                self.enable = True
            self.update_screen()
            return {
                self.status_codes["update redis"]: {
                    "redis key": "wheels enable",
                    "value": 1 if self.enable else 0,
                }
            }

        def front_left():
            """ Set speed for front left wheel """
            return {
                self.status_codes["edit"]: {
                    "param": self.wheel_speeds,
                    "setting": FRONT_LEFT,
                    "delta": 0.5,
                    "redis key": "fl wheel speed",
                }
            }

        def front_right():
            """ Set speed for front right wheel """
            return {
                self.status_codes["edit"]: {
                    "param": self.wheel_speeds,
                    "setting": FRONT_RIGHT,
                    "delta": 0.5,
                    "redis key": "fr wheel speed",
                }
            }

        def back_left():
            """ Set speed for back left wheel """
            return {
                self.status_codes["edit"]: {
                    "param": self.wheel_speeds,
                    "setting": BACK_LEFT,
                    "delta": 0.5,
                    "redis key": "bl wheel speed",
                }
            }

        def back_right():
            """ Set speed for back right wheel """
            return {
                self.status_codes["edit"]: {
                    "param": self.wheel_speeds,
                    "setting": BACK_RIGHT,
                    "delta": 0.5,
                    "redis key": "br wheel speed",
                }
            }

        # Listing actions for Config Wheels Screen
        self.actions = ["Set Wheel Speed", "fLeft", "fRight", "bLeft", "bRight", "Menu"]
        self.action_map = {
            self.actions[0]: set_wheel_speed,
            self.actions[1]: front_left,
            self.actions[2]: front_right,
            self.actions[3]: back_left,
            self.actions[4]: back_right,
            self.actions[5]: menu,
        }

        def draw_screen():
            """ Wheels Screen Layout """
            self.lcd_display.prepare()

            # Displaying the cursor
            cursor = ">"
            cursor_size = self.font.getsize(cursor)[0]
            cursor_pos_x = 0
            if self.curr_action != len(self.actions)-1:
                cursor_pos_y = BASE_Y + self.font_size * self.curr_action
            else:
                cursor_pos_y = self.lcd_display.height - self.font_size - PADDING

            self.lcd_display.draw.text(
                (cursor_pos_x, cursor_pos_y), cursor, font=self.font, fill="#ffffff"
            )

            # x and y coordinates for drawing on screen
            x = cursor_size
            y = BASE_Y

            set_wheel_speed_str = "Set Wheel Speed: "
            self.lcd_display.draw.text(
                (x, y), set_wheel_speed_str, font=self.font, fill="#ffffff"
            )
            x += self.font.getsize(set_wheel_speed_str)[0]
            self.lcd_display.draw.text(
                (x, y),
                "{}".format(self.enable),
                font=self.font,
                fill="#00ff00" if self.enable else "#0000ff",
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
                str(round(self.wheel_speeds[FRONT_LEFT], 1)),
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
                str(round(self.wheel_speeds[FRONT_RIGHT], 1)),
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
                str(round(self.wheel_speeds[BACK_LEFT], 1)),
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
                str(round(self.wheel_speeds[BACK_RIGHT], 1)),
                font=self.font,
                fill="#00ffff",
            )

            x = cursor_size
            y = (
                self.lcd_display.height - self.font_size - PADDING
            ) 
            self.lcd_display.draw.text(
                (x, y),
                "Go to Menu screen",
                font=self.font,
                fill="#ffffff",
            )

        # Pass Wheel Screen parameters to super class
        super().__init__(
            lcd_display, status_codes, self.actions, self.action_map, draw_screen
        )

    def update_values(self, redis_dict):
        """ Sync values with those from redis """
        if not self.edit_mode:
            self.enable = False if redis_dict["wheels enable"] == 0 else True
            self.wheel_speeds = [
                redis_dict["fl wheel speed"],
                redis_dict["fr wheel speed"],
                redis_dict["bl wheel speed"],
                redis_dict["br wheel speed"],
            ]