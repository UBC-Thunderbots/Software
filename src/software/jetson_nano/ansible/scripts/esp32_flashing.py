import Jetson.GPIO as GPIO
from time import sleep
import argparse


class FlashingESP32:
    PICO_BOOT_PIN = 15
    PICO_RESET_PIN = 13

    DELAY = 50 / 1000  # 50 MS

    def __init__(self):
        """Initialize GPIO pins"""
        GPIO.setmode(GPIO.BOARD)

        # set initial state to high to prevent ESP32 from entering bootloader mode
        GPIO.setup(self.PICO_BOOT_PIN, GPIO.OUT, initial=GPIO.HIGH)
        # set initial state to high to prevent ESP32 from resetting
        GPIO.setup(self.PICO_RESET_PIN, GPIO.OUT, initial=GPIO.HIGH)

    def enter_bootloader_mode(self):
        GPIO.output(self.PICO_BOOT_PIN, GPIO.HIGH)
        GPIO.output(self.PICO_RESET_PIN, GPIO.HIGH)
        sleep(self.DELAY)
        GPIO.output(self.PICO_BOOT_PIN, GPIO.LOW)
        sleep(self.DELAY)
        GPIO.output(self.PICO_RESET_PIN, GPIO.LOW)
        sleep(self.DELAY)
        GPIO.output(self.PICO_RESET_PIN, GPIO.HIGH)
        sleep(self.DELAY)
        GPIO.output(self.PICO_BOOT_PIN, GPIO.HIGH)

    def reset(self):
        GPIO.output(self.PICO_BOOT_PIN, GPIO.HIGH)
        GPIO.output(self.PICO_RESET_PIN, GPIO.HIGH)
        sleep(self.DELAY)
        GPIO.output(self.PICO_RESET_PIN, GPIO.LOW)
        sleep(self.DELAY)
        GPIO.output(self.PICO_RESET_PIN, GPIO.HIGH)


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
