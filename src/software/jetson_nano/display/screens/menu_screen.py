from screen import Screen

class MenuScreen(Screen):
    def __init__(self, lcd_display, key_list):
        # Defining this screens actions 
        def home():
            """ Action to go to Home Screen """
            return {self.key_list["change screen"]: "Home"}

        def config_wheels():
            """ Go to Wheel Screen """
            return {self.key_list["change screen"]: "Wheels"}

        def config_chip_and_kick():
            """ Go to Chip and Kick Screen """
            return {self.key_list["change screen"]: "Chip and Kick"}
       
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

            val0 = ">" if self.curr_action == 0 else " "
            val1 = ">" if self.curr_action == 1 else " "
            val2 = ">" if self.curr_action == 2 else " "

            x = 0
            y = 20
            
            self.lcd_display.draw.text((x, y), "{} Configure Wheels".format(val0), font=self.font, fill="#ffffff")
            y += self.font_size
            self.lcd_display.draw.text((x, y), "{} Configure Chip & Kick".format(val1), font=self.font, fill="#ffffff")

            y = self.lcd_display.height - self.font_size - 6
            self.lcd_display.draw.text((x, y), "{} Go to Home screen".format(val2), font=self.font, fill="#ffffff")

        # Pass Menu Screen parameters to super class
        super().__init__(lcd_display, key_list, self.actions, self.action_map, draw_screen)
    
if __name__ == "__main__":
    menu_screen = MenuScreen(None)
    menu_screen.on_click()