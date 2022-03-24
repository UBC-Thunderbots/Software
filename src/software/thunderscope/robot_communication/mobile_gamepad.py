from subprocess import Popen
import os
import pygame
import time

from proto.import_all_protos import *

from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.networking.threaded_unix_sender import ThreadedUnixSender


class MobileGamepad(object):
    def __init__(self, runtime_dir="/tmp/tbots"):

        """Runs our standalone er-force simulator binary and sets up the unix
        sockets to communicate with it

        :param runtime_dir: The runtime directory

        """
        pygame.init()
        self.mobile_gamepad = Popen(["/opt/mobile-gamepad/app.sh"])
        time.sleep(2)

        j = pygame.joystick.Joystick(0)
        j.init()

    def process_event(self):
        try:
            while True:
                events = pygame.event.get()
                for event in events:
                    if event.type == pygame.JOYAXISMOTION:
                        print(int(self.j.get_axis(0) * 100))
                        print(int(self.j.get_axis(1) * 100))

                    if event.type == pygame.JOYBUTTONDOWN:
                        print("Button Pressed")
                        if self.j.get_button(6):
                            print("button 6")
                            # Control Left Motor using L2
                        elif self.j.get_button(7):
                            print("button 7")
                            # Control Right Motor using R2
                    elif event.type == pygame.JOYBUTTONUP:
                        print("Button Released")

        except KeyboardInterrupt:
            self.j.quit()
