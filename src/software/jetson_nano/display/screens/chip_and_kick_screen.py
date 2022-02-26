from screen import Screen

CHIP = 0
KICK = 1


class ChipAndKickScreen(Screen):
    def __init__(self, lcd_display, status_codes):
        self.enable = False
        self.speeds = [
            0.0,
            0.0,
        ]  # [chip_speed, kick_speed] (using a list to mimic pass by reference)

        # Defining this screens actions
        def menu():
            """ Go to the menu screen """
            self.curr_action = 0

            return {self.status_codes["change screen"]: "Menu"}

        def set_chip_and_kick_speed():
            """ Enable and disable settings """
            if self.enable:
                self.enable = False
            else:
                self.enable = True

            self.update_screen()
            return {self.status_codes["none"]: None}

        def set_chip():
            """ Set chip speed """
            return {
                self.status_codes["edit"]: {
                    "param": self.speeds,
                    "setting": CHIP,
                    "delta": 0.5,
                }
            }

        def set_kick():
            """ Set kick speed """
            return {
                self.status_codes["edit"]: {
                    "param": self.speeds,
                    "setting": KICK,
                    "delta": 0.5,
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

            val0 = ">" if self.curr_action == 0 else " "
            val1 = ">" if self.curr_action == 1 else " "
            val2 = ">" if self.curr_action == 2 else " "
            val3 = ">" if self.curr_action == 3 else " "

            x = 0
            y = 20

            set_chip_and_kick_str = "{} Set Chip & Kick: ".format(val0)
            self.lcd_display.draw.text(
                (x, y), set_chip_and_kick_str, font=self.font, fill="#ffffff"
            )
            x = self.font.getsize(set_chip_and_kick_str)[0]
            self.lcd_display.draw.text(
                (x, y),
                "{}".format(self.enable),
                font=self.font,
                fill="#00ff00" if self.enable else "#0000ff",
            )

            x = 0
            y += self.font_size
            chip_str = "{} Chip Speed: ".format(val1)
            self.lcd_display.draw.text((x, y), chip_str, font=self.font, fill="#ffffff")
            x = (self.font.getsize(chip_str))[0]
            self.lcd_display.draw.text(
                (x, y), str(round(self.speeds[CHIP], 1)), font=self.font, fill="#00ffff"
            )

            x = 0
            y += self.font_size
            kick_str = "{} Kick Speed: ".format(val2)
            self.lcd_display.draw.text((x, y), kick_str, font=self.font, fill="#ffffff")
            x = (self.font.getsize(kick_str))[0]
            self.lcd_display.draw.text(
                (x, y), str(round(self.speeds[KICK], 1)), font=self.font, fill="#00ffff"
            )

            x = 0
            y = self.lcd_display.height - self.font_size - 6  # 6 is padding
            self.lcd_display.draw.text(
                (x, y),
                "{} Go to Menu screen".format(val3),
                font=self.font,
                fill="#ffffff",
            )

        # Pass Wheel Screen parameters to super class
        super().__init__(
            lcd_display, status_codes, self.actions, self.action_map, draw_screen
        )
