from screen import Screen

PADDING = 6
BASE_Y = 20

"""
This screen is used to navigate between different configuration settings.
"""
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
            self.actions[2]: home
        }
        def draw_screen():
            """ Menu Screen Layout """
            self.lcd_display.prepare()

            # displaying the cursor
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

            self.lcd_display.draw.text(
                (x, y),
                "Configure Wheels",
                font=self.font,
                fill="#ffffff",
            )
            y += self.font_size
            self.lcd_display.draw.text(
                (x, y),
                "Configure Chip & Kick".format(),
                font=self.font,
                fill="#ffffff",
            )

            y = (
                self.lcd_display.height - self.font_size - PADDING
            ) 
            self.lcd_display.draw.text(
                (x, y),
                "Go to Home screen",
                font=self.font,
                fill="#ffffff",
            )

        # Pass Menu Screen parameters to super class
        super().__init__(lcd_display, status_codes, self.actions, self.action_map, draw_screen)
    
if __name__ == "__main__":
    menu_screen = MenuScreen(None)
    menu_screen.on_click()
    