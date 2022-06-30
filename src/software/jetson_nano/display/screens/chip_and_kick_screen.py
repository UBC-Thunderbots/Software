from software.jetson_nano.display.screens.screen import Screen


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
                "redis key": "chip enable",
                "value": redis_dict["chip enable"],
                "type": bool,
                "delta": None,
                "screen action": screen_actions.UPDATE_REDIS,
                "display string": "Set Chip: ",
            },
            {
                "redis key": "kick enable",
                "value": redis_dict["kick enable"],
                "type": bool,
                "delta": None,
                "screen action": screen_actions.UPDATE_REDIS,
                "display string": "Set Kick: ",
            },
            {
                "redis key": "chip speed",
                "value": redis_dict["chip speed"],
                "type": float,
                "delta": 0.5,
                "screen action": screen_actions.EDIT_SCREEN,
                "display string": "Chip Speed [m/s]: ",
            },
            {
                "redis key": "kick speed",
                "value": redis_dict["kick speed"],
                "type": float,
                "delta": 0.5,
                "screen action": screen_actions.EDIT_SCREEN,
                "display string": "Kick Speed [m/s]: ",
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
