import subprocess
import time
import sys
from gpiozero import LED
from rich import print

# The reset pin for the motor drivers do i even need this?
MOTOR_DRIVER_RESET_PIN = 12

S0_PIN = 23
S1_PIN = 24

SWD_DISABLE_PIN = 0 # Pull HIGH to disable SWD multiplexing
SWCLK_DISABLE_PIN = 1 # Pull HIGH to disable SWCLK multiplexing

# Big thing is that Depending on the arguments you flash them. 

class MotorDriverFlasher:
    def __init__(self, board_letter, drivers):
        # A: (S0’ . S1’), B: (S0 . S1’), C: (S0’ . S1), D: (S0 . S1)
        # I know this is not very readable but it was a fun exercise
        # 0 -> 0 0
        # 1 -> 1 0
        # 2 -> 0 1
        # 3 -> 1 1
        switch_case_num = ord(board_letter) - ord('A')
        self.board_letter = board_letter
        self.multiplex = [switch_case_num % 2, switch_case_num // 2]

        self.drivers = drivers


    def flash(self):
        for i in range(2):
            if self.multiplex[i] == 1:
                self.drivers[i].on()
            else:
                self.drivers[i].off()

        print(
            f"Preparing to flash driver on board {self.board_letter}..."
        )


        # Short delay to ensure lines settle
        time.sleep(.5)

        try:
            # Run OpenOCD
            result = subprocess.run(
                [
                    "sudo", "openocd",
                    "-c", f"set RESET_PIN {MOTOR_DRIVER_RESET_PIN}",
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
        print(f"Flash to board {self.board_letter} complete.")
        # Let line settle rq
        time.sleep(0.5)


if __name__ == "__main__":
    # If no arguments given, 
    if not (2 <= len(sys.argv) <= 5):
        print("Usage: python3 flash_motor_drivers.py <flash_board_letter_1> <flash_board_letter_2> (A to D valid)")
        sys.exit(1)
    
    # Check before attempting to flash if every argument is "A" "B" "C" or "D"
    for i in range(1, len(sys.argv)):
        if sys.argv[i] not in ['A', 'B', 'C', 'D']:
            print("Usage: python3 flash_motor_drivers.py <flash_board_letter_1> <flash_board_letter_2> (A to D valid)")
            sys.exit(1)

    # Initialize pins as LEDs since we only need high/low logic.
    drivers = [LED(S0_PIN), LED(S1_PIN)]

    # Enable the multiplexers
    SWD_DISABLE = LED(SWD_DISABLE_PIN)
    SWCLK_DISABLE = LED(SWCLK_DISABLE_PIN)
    SWD_DISABLE.off()
    SWCLK_DISABLE.off()


    for i in range(1, len(sys.argv)):

        flasher = MotorDriverFlasher(sys.argv[i], drivers)
        flasher.flash()

    # Once done flashing, disable multiplexing just in case
    SWD_DISABLE.on()
    SWCLK_DISABLE.on()
