# How to build and flash the robot and dongle (radio) firmware

## The Robot
To simply build the code, run the `build_robot.sh` script

To build and flash the code (actually load it onto the robot), do the following:
1. Make sure the robot is turned off (but has a battery)
2. On the back of the robot, on the middle electrical board, there is an array of small switches. Flip the leftmost switch (labeled BL) down. This puts the robot in "boot mode"
3. Flick the power switch to turn on the robot, AND HOLD THE SWITCH IN THE "ON" POSITION
4. While still holding the power switch, run the `flash_robot.sh` script
5. Wait until the flashing script completes
6. Release the power switch on the robot
7. Turn the robot off
8. Flip the rightmost switch back to the "up" position
9. You can now power on the robot, and it should be running the new firmware!

## The Radio dongle
To simply buildthe code, run the `build_radio.sh` script

To build and flash the code (actually load it onto the dongle), do the following:
1. Make sure the dongle is unplugged from your comupter (so that it is unpowered)
2. Press and hold down the big red or yellow square button in the middle of the radio dongle board
3. While still holding the button down, plug in the dongle
4. You can now release the button. The dongle should now have a single red LED lit up. This indicates it is in "boot mode"
5. Run the `flash_dongle.sh` script. Wait for the script to complete
6. Unplug the dongle to shut it down
7. Plug the dongle back in (no need to hold the button again). The dongle should power up and be running the new firmware!

