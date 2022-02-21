from screen import Screen

class HomeScreen(Screen):
    def __init__(self, lcd_display, key_list):
        def menu():
            """ Action to go to Menu Screen """
            return {self.key_list["change screen"]: "Menu"}

        def robot_id():
            """ Set Robot ID """
            return {self.key_list["none"]: None}

        def channel_id():
            """ Set Channel ID """
            return {self.key_list["none"]: None}

        # Listing actions for Home Screen
        self.actions = ["Robot ID", "Channel ID", "Menu"]
        self.action_map = {
            self.actions[0]: robot_id,
            self.actions[1]: channel_id,
            self.actions[2]: menu
        }

        def draw_screen():
            """ Home Screen Layout """
            self.lcd_display.prepare()
        
            val0 = ">" if self.curr_action == 0 else " "
            val1 = ">" if self.curr_action == 1 else " "
            val2 = ">" if self.curr_action == 2 else " "
        
            robot_id_str = "{} Robot ID".format(val0)
            self.lcd_display.draw.text((0, 20), robot_id_str, font=self.font, fill="#FFFFFF")
            self.lcd_display.draw.text((self.font.getsize(robot_id_str)[0]+3, 20), "{} Channel ID".format(val1), font=self.font, fill="#FFFFFF") 

            battery_str = "  Battery Voltage: "
            self.lcd_display.draw.text((0, 48+8), battery_str, font=self.font, fill="#FFFFFF")
            
            capacitor_str = "  Capacitor Voltage: "
            self.lcd_display.draw.text((0, 60+8), capacitor_str, font=self.font, fill="#FFFFFF")

            packet_loss_str = "  Packet Loss %: " 
            self.lcd_display.draw.text((0, 72+8), packet_loss_str, font=self.font, fill="#FFFFFF")
            
            self.lcd_display.draw.text((0, self.lcd_display.height-6-self.font_size), "{} Go to Menu screen".format(val2), font=self.font, fill="#FFFFFF")
            
        # Pass Home Screen parameters to super class
        super().__init__(lcd_display, key_list, self.actions, self.action_map, draw_screen)

# For testing
if __name__ == "__main__":
    import sys
    sys.path.append('../')
    from lcd_user_interface import LcdDisplay
    home_screen = HomeScreen(LcdDisplay())
    home_screen.on_clockwise_rotate()