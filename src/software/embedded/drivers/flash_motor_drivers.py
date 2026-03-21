import subprocess
import time
import sys
from gpiozero import LED
from rich import print

# The reset pins for the motor drivers
MOTOR_DRIVER_RESET_PINS = [23, 12]
# MOTOR_DRIVER_FLASH_RESET_PIN = 12

class MotorDriverFlasher:
    def __init__(self, pins, flashpin):
        # Initialize pins as LEDs since we only need high/low logic.
        # We assume High = Run/Active, Low = Reset.
        self.flashpin = flashpin
        self.drivers = [LED(pin) for pin in pins if pin != flashpin]

    def flash(self):
        # Pull other drivers Low = REset
        for i, other_driver in enumerate(self.drivers):
            other_driver.off()

        print(
            f"Preparing to flash driver on pin {self.flashpin}..."
        )


        # Short delay to ensure lines settle
        time.sleep(1)

        try:
            # Run OpenOCD
            result = subprocess.run(
                [
                    "sudo", "openocd",
                    "-c", f"set RESET_PIN {self.flashpin}",
                    "-f", "stm32_rpi.cfg",
                    # Force the Pi to hold the reset line down during connection
                    "-c", "reset_config srst_only srst_nogate connect_assert_srst",
                    "-c", "adapter srst delay 100", 
                    "-c", "init",
                    # Modern atomic command
                    "-c", "program mdv6_firmware_main.bin verify reset exit 0x08000000",
                ],
                capture_output=True,
                text=True,
                check=True,
            )
            print(
                f"Flash output for driver:\n{result.stderr}"
            )  # OpenOCD often prints to stderr
        except subprocess.CalledProcessError as e:
            print("Failed to flash driver!")
            print("STDOUT:", e.stdout)
            print("STDERR:", e.stderr)
            # We raise to stop the process if one fails, or we could continue.
            # Usually best to know immediately.
            raise e

        # After flashing all, ensure all are set to High (Run)
        print("Flashing complete. Setting all drivers to RUN state.")
        for driver in self.drivers:
            driver.off()
            time.sleep(0.5)
            driver.on()
        print("Reset complete.")


if __name__ == "__main__":
    # If no arguments given, 
    if len(sys.argv) < 2:
        print("Usage: python3 flash_motor_drivers.py <flash_pin>")
        sys.exit(1)

    flash_pin = int(sys.argv[1])
    flasher = MotorDriverFlasher(MOTOR_DRIVER_RESET_PINS, flash_pin)
    flasher.flash()
