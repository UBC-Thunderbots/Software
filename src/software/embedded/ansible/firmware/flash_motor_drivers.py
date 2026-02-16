import subprocess
from gpiozero import LED


class MotorDriver:
    MOTOR_DRIVER_RESET_PINS = [5, 6, 7, 8]

    """Contains basic information about a motor driver board's physical information"""
    def __init__(self, reset_gpio):
        """
        Creates a motor driver object which contains the information about a particular board's wiring
        :param reset_gpio: The pin which resets the driver board
        """
        self.reset_gpio = LED(reset_gpio)

    def flash(self):
        self.reset_gpio.on()

        result = subprocess.run([
            "openocd",
            "-f", "interface/raspberrypi-swd.cfg",
            "-f", "target/stm32f0x.cfg",
            "-c", "program mdv6_firmware_main.bin verify reset exit 0x08000000"
        ], capture_output=True, text=True, check=True)
        print("Flash output:", result.stdout)


if __name__ == "__main__":
    pins = []
    for pin in MotorDriver.MOTOR_DRIVER_RESET_PINS:
        pins.append(LED(pin))
