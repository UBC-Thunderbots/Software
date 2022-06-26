from software.jetson_nano.display.screens.screen import Screen


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
                "screen action": screen_actions.UPDATE_REDIS,
                "display string": "Set Wheel Speed: ",
            },
            {
                "redis key": "fl wheel speed",
                "value": redis_dict["fl wheel speed"],
                "type": float,
                "delta": 0.5,
                "screen action": screen_actions.EDIT_SCREEN,
                "display string": "Front Left [m/s]: ",
            },
            {
                "redis key": "fr wheel speed",
                "value": redis_dict["fr wheel speed"],
                "type": float,
                "delta": 0.5,
                "screen action": screen_actions.EDIT_SCREEN,
                "display string": "Front Right [m/s]: ",
            },
            {
                "redis key": "bl wheel speed",
                "value": redis_dict["bl wheel speed"],
                "type": float,
                "delta": 0.5,
                "screen action": screen_actions.EDIT_SCREEN,
                "display string": "Back Left [m/s]: ",
            },
            {
                "redis key": "br wheel speed",
                "value": redis_dict["br wheel speed"],
                "type": float,
                "delta": 0.5,
                "screen action": screen_actions.EDIT_SCREEN,
                "display string": "Back Right [m/s]: ",
            },
            {
                "redis key": None,
                "value": "Menu",
                "type": str,
                "delta": None,
                "screen action": screen_actions.CHANGE_SCREEN,
                "display string": "Go to Menu screen",
            },
        ]

        # Pass Wheel Screen parameters to super class
        super().__init__(lcd_display, screen_actions, actions)
