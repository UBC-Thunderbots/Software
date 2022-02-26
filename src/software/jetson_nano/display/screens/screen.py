from PIL import ImageFont
import subprocess

class Screen():
    def __init__(self, lcd_display, status_codes, actions, action_map, draw_screen): 
        self.lcd_display = lcd_display
        self.font_size = 12
        self.font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', self.font_size)
        self.status_codes = status_codes
        self.draw_screen = draw_screen

        # Screen actions
        self.actions = actions
        self.action_map = action_map

        # Maintain current action        
        self.len = len(self.actions)
        self.curr_action = 0

        self.edit_mode = False
        self.param = None
        self.setting = None
        self.delta = None

    def draw_header(self):
        """ Draw the display header """
        self.lcd_display.draw.rectangle((0, 0, self.lcd_display.width, self.font_size+6), outline="#000000", fill="#ffffff")
        
        cmd = "hostname -I | cut -d\' \' -f1"
        IP = "  IP: "+subprocess.check_output(cmd, shell=True).decode("utf-8")
        
        self.lcd_display.draw.text((0, 2), IP, font=self.font, fill="#000000")
        
        cmd = "iwconfig | grep 'Signal level='"
        signal = " " + subprocess.check_output(cmd, stderr=subprocess.STDOUT, shell=True).decode("utf-8").split("Signal level=")[1].replace("\n",'')

        self.lcd_display.draw.text((self.lcd_display.width/2+24, 2), signal, font=self.font, fill="#000000")

        self.lcd_display.show()

    def update_screen(self):
        """ Draw the updated screen """
        self.draw_screen()
        self.lcd_display.show()
        self.draw_header()

    def on_click(self):
        """ Execute the current action """
        action = self.action_map[self.actions[self.curr_action]]()
        
        # For editing settings
        if self.status_codes["edit"] in action: 
            payload = action[self.status_codes["edit"]]
            
            self.param = payload["param"]
            self.setting = payload["setting"]
            self.delta = payload["delta"]
            self.edit_mode = not self.edit_mode

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
        self.param[self.setting] += self.delta
        
    def dec_val(self):
        """ Decrement self.param by self.delta """
        self.param[self.setting] -= self.delta

