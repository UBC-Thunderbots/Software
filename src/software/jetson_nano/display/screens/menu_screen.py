from screen import Screen


class MenuScreen(Screen):
    def __init__(self, lcd_display, status_codes):
        # Defining this screens actions
        def home():
            """ Action to go to Home Screen """
            return {self.status_codes["change screen"]: "Home"}

        def config_wheels():
            """ Go to Wheel Screen """
            return {self.status_codes["change screen"]: "Wheels"}

        def config_chip_and_kick():
            """ Go to Chip and Kick Screen """
            return {self.status_codes["change screen"]: "Chip and Kick"}

        # Listing actions for Menu Screen
        self.actions = ["Wheels", "Chip and Kick", "Home"]
        self.action_map = {
            self.actions[0]: config_wheels,
            self.actions[1]: config_chip_and_kick,
            self.actions[2]: home,
        }

        def draw_screen():
            """ Menu Screen Layout """
            self.lcd_display.prepare()

            """
            TODO: remove these after testing new cursors
            val0 = ">" if self.curr_action == 0 else " "
            val1 = ">" if self.curr_action == 1 else " "
            val2 = ">" if self.curr_action == 2 else " "
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

            self.lcd_display.draw.text(
                (x, y),
                "Configure Wheels",
                #"{} Configure Wheels".format(val0),
                font=self.font,
                fill="#ffffff",
            )
            y += self.font_size
            self.lcd_display.draw.text(
                (x, y),
                "Configure Chip & Kick".format(),
                #"{} Configure Chip & Kick".format(val1),
                font=self.font,
                fill="#ffffff",
            )

            y = self.lcd_display.height - self.font_size - 6 # TODO: define this value, its for padding
            self.lcd_display.draw.text(
                (x, y),
                "Go to Home screen",
                #"{} Go to Home screen".format(val2),
                font=self.font,
                fill="#ffffff",
            )

        # Pass Menu Screen parameters to super class
        super().__init__(
            lcd_display, status_codes, self.actions, self.action_map, draw_screen
        )


if __name__ == "__main__":
    menu_screen = MenuScreen(None)
    menu_screen.on_click()
