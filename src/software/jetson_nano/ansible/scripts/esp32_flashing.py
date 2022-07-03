import Jetson.GPIO as GPIO
from time import sleep
import argparse


class FlashingESP32:
    PICO_BOOT = 11
    PICO_RESET = 18
    DELAY_S = 50 / 1000

    def __init__(self):
        """Initialize GPIO pins"""
        GPIO.setmode(GPIO.BOARD)

        # set initial state to high to prevent ESP32 from entering bootloader mode
        GPIO.setup(self.PICO_BOOT, GPIO.OUT, initial=GPIO.HIGH)
        # set initial state to high to prevent ESP32 from reseting
        GPIO.setup(self.PICO_RESET, GPIO.OUT, initial=GPIO.HIGH)

    def enter_bootloader_mode(self):
        GPIO.output(self.PICO_BOOT, GPIO.HIGH)
        GPIO.output(self.PICO_RESET, GPIO.HIGH)
        sleep(self.DELAY_S)
        GPIO.output(self.PICO_BOOT, GPIO.LOW)
        sleep(self.DELAY_S)
        GPIO.output(self.PICO_RESET, GPIO.LOW)
        sleep(self.DELAY_S)
        GPIO.output(self.PICO_RESET, GPIO.HIGH)
        sleep(self.DELAY_S)
        GPIO.output(self.PICO_BOOT, GPIO.HIGH)

    def reset(self):
        GPIO.output(self.PICO_BOOT, GPIO.HIGH)
        GPIO.output(self.PICO_RESET, GPIO.HIGH)
        sleep(self.DELAY_S)
        GPIO.output(self.PICO_RESET, GPIO.LOW)
        sleep(self.DELAY_S)
        GPIO.output(self.PICO_RESET, GPIO.HIGH)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=["reset", "bootloader"], help="command to run")
    args, unknown_args = parser.parse_known_args()

    esp32 = FlashingESP32()

    if args.command in "reset":
        esp32.reset()
        print("reset done")
    if args.command in "bootloader":
        esp32.enter_bootloader_mode()
        print("entered bootloader")
