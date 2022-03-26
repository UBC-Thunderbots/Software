from screen import Screen

ENABLE_INDEX = 0
FL_INDEX = 1
FR_INDEX = 2
BL_INDEX = 3
BR_INDEX = 4


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
                "display string": "Front Left: ",
            },
            {
                "redis key": "fr wheel speed",
                "value": redis_dict["fr wheel speed"],
                "type": float,
                "delta": 0.5,
                "screen action": screen_actions.EDIT_SCREEN,
                "display string": "Front Right: ",
            },
            {
                "redis key": "bl wheel speed",
                "value": redis_dict["bl wheel speed"],
                "type": float,
                "delta": 0.5,
                "screen action": screen_actions.EDIT_SCREEN,
                "display string": "Back Left: ",
            },
            {
                "redis key": "br wheel speed",
                "value": redis_dict["br wheel speed"],
                "type": float,
                "delta": 0.5,
                "screen action": screen_actions.EDIT_SCREEN,
                "display string": "Back Right: ",
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

    def update_values(self, redis_dict):
        """ Sync values with those from redis """
        if not self.edit_mode:
            self.actions[ENABLE_INDEX]["value"] = (
                1 if redis_dict["wheels enable"] else 0
            )
            self.actions[FL_INDEX]["value"] = redis_dict["fl wheel speed"]
            self.actions[FR_INDEX]["value"] = redis_dict["fr wheel speed"]
            self.actions[BL_INDEX]["value"] = redis_dict["bl wheel speed"]
            self.actions[BR_INDEX]["value"] = redis_dict["br wheel speed"]
