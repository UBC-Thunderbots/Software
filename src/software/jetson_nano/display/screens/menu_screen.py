from screen import Screen

PADDING = 6
BASE_Y = 20


class MenuScreen(Screen):
    """
    This screen is used to navigate between different configuration settings.
    """
    def __init__(self, lcd_display, screen_actions):
        """
        @param lcd_display, an instance of the LcdDisplay class
        @param screen_actions, an instance of ScreenActions class
        """
        actions = [              
            {
                "redis key": None,
                "value": "Wheels",
                "type": str,
                "delta": None,
                "screen action": screen_actions.CHANGE_SCREEN
            },
            {
                "redis key": None,
                "value": "Chip and Kick",
                "type": str,
                "delta": 0.5,
                "screen action": screen_actions.CHANGE_SCREEN
            },
            {
                "redis key": None,
                "value": "Home",
                "type": str,
                "delta": None,
                "screen action": screen_actions.CHANGE_SCREEN,
            },
        ]

        def draw_screen():
            """ Menu Screen Layout """
            self.lcd_display.prepare()

            # displaying the cursor
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

            self.lcd_display.draw.text(
                (x, y), "Configure Wheels", font=self.font, fill="#ffffff",
            )
            y += self.font_size
            self.lcd_display.draw.text(
                (x, y),
                "Configure Chip & Kick".format(),
                font=self.font,
                fill="#ffffff",
            )

            y = self.lcd_display.height - self.font_size - PADDING
            self.lcd_display.draw.text(
                (x, y), "Go to Home screen", font=self.font, fill="#ffffff",
            )

        # Pass Menu Screen parameters to super class
        super().__init__(lcd_display, screen_actions, draw_screen, actions)


if __name__ == "__main__":
    menu_screen = MenuScreen(None)
    menu_screen.on_click()
