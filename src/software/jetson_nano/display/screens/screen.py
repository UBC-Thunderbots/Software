from PIL import ImageFont

import software.jetson_nano.display.constants as constants
from software.jetson_nano.display.utils import get_ip_address, get_signal_strength


class Screen:
    """
    All new screens will inherit from this class. This class will handles editing variables and maintaining
    the current action the cursor is hovering. It also accepts the list of actions so that it can display
    them on the screen.
    """

    def __init__(self, lcd_display, screen_actions, actions, draw_screen=None):
        """
        @param lcd_display, an instance of the LcdDisplay class
        @param screen_actions, an instance of ScreenActions class
        @param draw_screen, a callback function to re-render screen on lcd display
        @param actions, a list of dictionaries for the interactive elements on the screen
        """
        self.actions = actions
        self.edit_mode = False
        self.action = None

        self.screen_actions = screen_actions
        self.draw_screen = draw_screen

        # Maintain current action
        self.len = len(self.actions)
        self.curr_action = 0

        self.lcd_display = lcd_display
        self.font_size = 11
        self.font = ImageFont.truetype("DejaVuSans.ttf", self.font_size)
        self.big_font_size = 22
        self.big_font = ImageFont.truetype("DejaVuSans.ttf", self.big_font_size)

    def draw_header(self):
        """ Draw the display header """
        self.lcd_display.draw.rectangle(
            (0, 0, self.lcd_display.width, self.font_size + 6),
            outline=constants.BLACK,
            fill=constants.WHITE,
        )

        # Get IP address
        IP = " IP: " + get_ip_address()
        self.lcd_display.draw.text((0, 2), IP, font=self.font, fill=constants.BLACK)

        # Get signal strength
        signal_strength = " " + get_signal_strength()
        self.lcd_display.draw.text(
            (self.lcd_display.width - self.font.getsize(signal_strength)[0], 2),
            signal_strength,
            font=self.font,
            fill=constants.BLACK,
        )

        self.lcd_display.show()

    def update_screen(self):
        """ Draw the updated screen """
        if self.draw_screen != None:
            self.draw_screen()
        else:
            self.draw_actions()
        self.lcd_display.show()
        self.draw_header()

    def on_click(self):
        """ Execute the current action """
        action = self.actions[self.curr_action]

        # For editing settings
        if self.screen_actions.EDIT_SCREEN == action["screen action"]:
            self.action = action
            if not self.edit_mode:
                action = {"screen action": self.screen_actions.NONE}
            else:
                action = {
                    "redis key": self.action["redis key"],
                    "value": self.action["value"],
                    "screen action": self.screen_actions.UPDATE_REDIS,
                }
            self.edit_mode = not self.edit_mode
        elif action["type"] == bool:
            action["value"] = 0 if action["value"] else 1
            self.update_screen()
        elif self.screen_actions.CHANGE_SCREEN == action["screen action"]:
            self.curr_action = 0

        return action

    def on_clockwise_rotate(self):
        """ Update current action and update screen """
        if not self.edit_mode:
            self.curr_action = (self.curr_action + 1) % self.len
        else:
            self.inc_val()

        self.update_screen()

    def on_counterclockwise_rotate(self):
        """ Update current action and update screen """
        if not self.edit_mode:
            self.curr_action = (self.curr_action - 1) % self.len
        else:
            self.dec_val()

        self.update_screen()

    def inc_val(self):
        """ Increment self.param by self.delta"""
        self.action["value"] += self.action["delta"]

    def dec_val(self):
        """ Decrement self.param by self.delta """
        self.action["value"] -= self.action["delta"]

    def draw_actions(self):
        """ Draws our list of actions """
        self.lcd_display.prepare()

        cursor = constants.CURSOR
        cursor_size = self.font.getsize(cursor)[0]
        cursor_pos_x = constants.BASE_X

        if self.curr_action != len(self.actions) - 1:
            cursor_pos_y = constants.BASE_Y + self.font_size * self.curr_action
        else:
            cursor_pos_y = self.lcd_display.height - self.font_size - constants.PADDING

        self.lcd_display.draw.text(
            (cursor_pos_x, cursor_pos_y), cursor, font=self.font, fill=constants.WHITE
        )

        x = cursor_size
        y = constants.BASE_Y

        for action in self.actions:
            if (
                action == self.actions[-1]
            ):  # last action should take you to previous screen
                y = self.lcd_display.height - self.font_size - constants.PADDING

            self.lcd_display.draw.text(
                (x, y), action["display string"], font=self.font, fill=constants.WHITE
            )

            # Display the value for an action; no value when changing screens
            if action["screen action"] != self.screen_actions.CHANGE_SCREEN:
                x += self.font.getsize(action["display string"])[0]

                if action["type"] == bool:
                    en = True if action["value"] else False
                    self.lcd_display.draw.text(
                        (x, y),
                        "{}".format(en),
                        font=self.font,
                        fill=constants.GREEN if en else constants.RED,
                    )
                else:
                    self.lcd_display.draw.text(
                        (x, y),
                        str(round(action["value"], 1)),
                        font=self.font,
                        fill=constants.YELLOW,
                    )
                x = cursor_size

            y += self.font_size

    def update_values(self, redis_dict):
        """ Sync values with those from redis """
        if not self.edit_mode:
            for i in range(self.len):
                if self.actions[i]["redis key"] == None:
                    continue

                if self.actions[i]["type"] == bool:
                    self.actions[i]["value"] = (
                        1 if redis_dict[self.actions[i]["redis key"]] else 0
                    )
                else:
                    self.actions[i]["value"] = redis_dict[self.actions[i]["redis key"]]
