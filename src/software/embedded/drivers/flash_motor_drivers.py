import subprocess
import time
from gpiozero import LED
from rich import print

# The reset pins for the motor drivers
# MOTOR_DRIVER_RESET_PINS = [23, 12]
MOTOR_DRIVER_RESET_PINS = [12]

class MotorDriverFlasher:
    def __init__(self, pins):
        # Initialize pins as LEDs since we only need high/low logic.
        # We assume High = Run/Active, Low = Reset.
        self.drivers = [LED(pin) for pin in pins]

    def flash_all(self):
        for i, target_driver in enumerate(self.drivers):
            print(
                f"Preparing to flash driver {i} on pin {MOTOR_DRIVER_RESET_PINS[i]}..."
            )

            # Set all other drivers to Low (Reset)
            for j, other_driver in enumerate(self.drivers):
                if i != j:
                    other_driver.off()

            # Set target driver to High (Active)
            target_driver.on()

            # Short delay to ensure lines settle
            time.sleep(1)

            print(f"Flashing driver {i}...")
            try:
                # Run OpenOCD
                result = subprocess.run(
                    [
                        "openocd",
                        "-f", "stm32_rpi.cfg",
                        "-c", "program mdv6_firmware_main verify reset exit",
                    ],
                    capture_output=True,
                    text=True,
                    check=True,
                )
                print(
                    f"Flash output for driver {i}:\n{result.stderr}"
                )  # OpenOCD often prints to stderr
            except subprocess.CalledProcessError as e:
                print(f"Failed to flash driver {i}!")
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
    flasher = MotorDriverFlasher(MOTOR_DRIVER_RESET_PINS)
    flasher.flash_all()
