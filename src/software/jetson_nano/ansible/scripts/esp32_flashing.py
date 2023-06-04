import Jetson.GPIO as GPIO
from time import sleep
import argparse


class FlashingESP32:
    PICO_BOOT_PIN = 11
    PICO_RESET_PIN = 12

    DELAY = 50 / 1000  #  50 MS

    def __init__(self):
        """Initialize GPIO pins"""
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)

        # set initial state to high to prevent ESP32 from entering bootloader mode
        GPIO.setup(self.PICO_BOOT_PIN, GPIO.OUT, initial=GPIO.LOW)
        # set initial state to high to prevent ESP32 from resetting
        GPIO.setup(self.PICO_RESET_PIN, GPIO.OUT, initial=GPIO.LOW)

    def enter_bootloader_mode(self):
        GPIO.output(self.PICO_BOOT_PIN, GPIO.LOW)
        GPIO.output(self.PICO_RESET_PIN, GPIO.LOW)
        sleep(self.DELAY)
        GPIO.output(self.PICO_BOOT_PIN, GPIO.HIGH)
        sleep(self.DELAY)
        GPIO.output(self.PICO_RESET_PIN, GPIO.HIGH)
        sleep(self.DELAY)
        GPIO.output(self.PICO_RESET_PIN, GPIO.LOW)
        sleep(self.DELAY)
        GPIO.output(self.PICO_BOOT_PIN, GPIO.LOW)
        GPIO.cleanup()

    def reset(self):
        GPIO.output(self.PICO_BOOT_PIN, GPIO.LOW)
        GPIO.output(self.PICO_RESET_PIN, GPIO.LOW)
        sleep(self.DELAY)
        GPIO.output(self.PICO_RESET_PIN, GPIO.HIGH)
        sleep(self.DELAY)
        GPIO.output(self.PICO_RESET_PIN, GPIO.LOW)
        GPIO.cleanup()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "command", choices=["reset", "bootloader"], help="command to run"
    )
    args, unknown_args = parser.parse_known_args()

    esp32 = FlashingESP32()

    if args.command in "reset":
        esp32.reset()
        print("reset done")
    if args.command in "bootloader":
        esp32.enter_bootloader_mode()
        print("entered bootloader")
