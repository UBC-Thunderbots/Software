from software.jetson_nano.display.screens.screen import Screen


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
                "screen action": screen_actions.CHANGE_SCREEN,
                "display string": "Configure Wheels",
            },
            {
                "redis key": None,
                "value": "Chip and Kick",
                "type": str,
                "delta": None,
                "screen action": screen_actions.CHANGE_SCREEN,
                "display string": "Configure Chip & Kick",
            },
            {
                "redis key": None,
                "value": "Home",
                "type": str,
                "delta": None,
                "screen action": screen_actions.CHANGE_SCREEN,
                "display string": "Go to Home screen",
            },
        ]

        # Pass Menu Screen parameters to super class
        super().__init__(lcd_display, screen_actions, actions)


if __name__ == "__main__":
    menu_screen = MenuScreen(None)
    menu_screen.on_click()
