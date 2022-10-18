# Common Robot Commands

# Table of Contents
* [Off Robot Commands](#off-robot-commands)
   * [Wifi Disclaimer](#wifi-disclaimer)
   * [Flashing the nano](#flashing-the-nano)
   * [Flashing the powerboard](#flashing-the-powerboard)
   * [Setting up nano](#setting-up-nano)
* [On Robot Commands](#on-robot-commands)
   * [Systemd Services](#systemd-services)
   * [Debugging Uart](#debugging-uart)
   * [Redis](#redis)

# Off Robot Commands

## Wifi Disclaimer

To use most of these commands you will either need to be on the tbots wifi network( no internet access ) or on a wifi with internet access(shawopen, ubc wifi) and connected to the jetson nano through ethernet tethering. 

The IP address of the robots on the tbots network is `192.168.0.20<robot_id>` so for robot id `1` the ip is `192.168.0.201`. If you are using ethernet tethering you will need to use a network utility (tshark, wireshark, arp) to determine the ip address.

## Flashing the nano

This will stop the current systemd services, replace and restart them. Binaries that are used for systemd services are located in a folder on home(the default directory) called thunderbots_binaries.

<b>To build this for the first time you will need to run this with internet access. Then run it again on the tbots network</b>

`bazel run //software/jetson_nano/ansible:run_ansible --cpu=jetson_nano -- -playbook deploy_nano.yml --hosts <robot_ip> --ssh_pass thunderbots`

## Flashing the powerboard

This will flash powerloop, the current firmware in `software/power/`, onto the power board. It will prompt the user into setting the powerboard into bootloader mode by holding the reset button and pressing the boot button. Then once the board is flashed, pressing the reset button after to use the new firmware.  

Looking from the back of the robot the reset and boot buttons are on right side of the battery holder on the lowest board with the reset being on the left and the boot on the right. <b>Warning it may kick/chip when pressed.</b>

`bazel run //software/jetson_nano/ansible:run_ansible --cpu=jetson_nano -- -playbook deploy_powerboard.yml --hosts <robot_ip> --ssh_pass thunderbots`

## Setting up nano 

This refers to setting up the jetson nano for the first time. This will enable systemd services, modify device tree files and perform other setup necessary for the communication protocols used.

<b>Setting up the nano for the first time requires internet access!!!</b>

`bazel run //software/jetson_nano/ansible:run_ansible --cpu=jetson_nano -- -playbook setup_nano.yml --hosts <robot_ip> --ssh_pass thunderbots`


# On Robot Commands

`ssh robot@<robot_ip>` can be used to connect to a robot.

## Systemd Services

Status shows whether the service is running and some recent logs. More logs can be found using `journctl` shown below.

`service <service_name> status`

Change whether the service is running or restart it. valid <run_command> are stop, start and restart.

`service <service_name> <run_command>`
 
To view the full logs in vi:  

`journalctl <service_name>`  

To follow the recent outputs to the log:

`journalctl -fu <service_name>`

## Debugging Uart

`screen` or [GNU screen](https://www.gnu.org/software/screen/) can act as serial monitor displaying what the jetson is receiving and allowing us to send characters through the terminal. 

For uart we use serial port: `/dev/ttyTHS1`  
For usb we use serial port: `/dev/ttyUSB0`  

The baudrate of powerloop is `460800` other programs will have different baudrates commonly `115200`. If the baudrate is incorrect you will see foreign characters.

Usage:

`screen <serial_port> <baudrate>`

Exiting:

`Ctrl-a` -> `k` -> `y` -> `enter`

Debugging:

If the serial_port is busy, screen will not launch and instead says `screen is terminating`. This may be because the port is in use either by thunderloop or another process running screen.

Powerloop uart communication is encoded so you cant read it from screen and will appear as a mix of foreign characters

Pressing the reset button once will send a status msg over its connected port. This is useful for sanity checking.

## Redis

Current redis keys that are used are available in `software/constants.h`.  Official Documentation [here](https://redis.io/docs/manual/cli/).

<b>Values should be strings. For example `set \ROBOT_ID "0"`</b>

Redis repl can be accessed through the following command.

`redis-cli`

Other common commands (once inside redis repl):

`get <redis_key> <value>`

`set <redis_key> <value>`

To Exit:

`quit`

Alternative (without entering redis repl):

`redis-cli get <redis_key> <value>`