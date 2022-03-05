from screen import Screen

FRONT_LEFT = 0
FRONT_RIGHT = 1
BACK_LEFT = 2
BACK_RIGHT = 3


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

            """
            TODO: remove these after testing the new cursors
            val0 = ">" if self.curr_action == 0 else " "
            val1 = ">" if self.curr_action == 1 else " "
            val2 = ">" if self.curr_action == 2 else " "
            val3 = ">" if self.curr_action == 3 else " "
            val4 = ">" if self.curr_action == 4 else " "
            val5 = ">" if self.curr_action == 5 else " "
            """

            # TODO: use this to put the cursor positions
            cursor_pos_x = 0
            cursor_pos_y = 20 + self.font_size * self.curr_action

            self.lcd_display.draw.text(
                (cursor_pos_x, cursor_pos_y), ">", font=self.font, fill="#ffffff"
            )

            # x and y coordinates for drawing on screen
            x = 3
            y = 20

            set_wheel_speed_str = "Set Wheel Speed: "
            # set_wheel_speed_str = "{} Set Wheel Speed: ".format(val0)
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

            x = 3
            y += self.font_size
            front_left_str = "Front Left: "
            # front_left_str = "{} Front Left: ".format(val1)
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

            x = 3
            y += self.font_size
            front_right_str = "Front Right: "
            # front_right_str = "{} Front Right: ".format(val2)
            self.lcd_display.draw.text(
                (x, y), front_right_str, font=self.font, fill="#ffffff"
            )
            x = self.font.getsize(front_right_str)[0]
            self.lcd_display.draw.text(
                (x, y),
                str(round(self.wheel_speeds[FRONT_RIGHT], 1)),
                font=self.font,
                fill="#00ffff",
            )

            x = 3
            y += self.font_size
            back_left_str = "Back Left: "
            # back_left_str = "{} Back Left: ".format(val3)
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

            x = 3
            y += self.font_size
            back_right_str = "Back Right: "
            # back_right_str = "{} Back Right: ".format(val4)
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

            x = 3
            y = (
                self.lcd_display.height - self.font_size - 6
            )  # TODO define this padding number
            self.lcd_display.draw.text(
                (x, y),
                "Go to Menu screen",
                # "{} Go to Menu screen".format(val5),
                font=self.font,
                fill="#ffffff",
            )

        # Pass Wheel Screen parameters to super class
        super().__init__(
            lcd_display, status_codes, self.actions, self.action_map, draw_screen
        )
