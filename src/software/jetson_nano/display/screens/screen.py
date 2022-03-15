from PIL import ImageFont
import subprocess


class Screen:
    """
    All new screens will inherit from this class. This class will handles editing variables and maintaining
    the current action the cursor is hovering.
    """
    def __init__(self, lcd_display, screen_actions, draw_screen, actions):
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
        self.font_size = 12
        self.font = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", self.font_size
        )
        self.big_font_size = 22
        self.big_font = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", self.big_font_size
        )

    def draw_header(self):
        """ Draw the display header """
        self.lcd_display.draw.rectangle(
            (0, 0, self.lcd_display.width, self.font_size + 6),
            outline="#000000",
            fill="#ffffff",
        )

        # Get IP address
        try:
            cmd = "hostname -I | cut -d' ' -f1"
            IP = "  IP: " + subprocess.check_output(cmd, shell=True).decode("utf-8")
        except:
            IP = "  IP: N/A"

        self.lcd_display.draw.text((0, 2), IP, font=self.font, fill="#000000")

        # Get signal strength
        try:
            cmd = "iwconfig | grep 'Signal level='"
            signal = " " + subprocess.check_output(
                cmd, stderr=subprocess.STDOUT, shell=True
            ).decode("utf-8").split("Signal level=")[1].replace("\n", "")
        except:
            signal = "N/A"

        self.lcd_display.draw.text(
            (self.lcd_display.width - self.font.getsize(signal)[0], 2),
            signal,
            font=self.font,
            fill="#000000",
        )

        self.lcd_display.show()

    def update_screen(self):
        """ Draw the updated screen """
        self.draw_screen()
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
                    "screen action": self.screen_actions.UPDATE_REDIS
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
