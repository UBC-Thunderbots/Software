# Useful Robot Commands

# Table of Contents

<!--TOC-->

- [Table of Contents](#table-of-contents)
- [Common Debugging Steps](#common-debugging-steps)
- [Off Robot Commands](#off-robot-commands)
  - [Wifi Disclaimer](#wifi-disclaimer)
  - [Miscellaneous Ansible Tasks & Options](#miscellaneous-ansible-tasks--options)
  - [Flashing the robot's compute module](#flashing-the-robots-compute-module)
  - [Flashing the powerboard](#flashing-the-powerboard)
  - [Setting up the embedded host](#setting-up-the-embedded-host)
    - [Raspberry Pi](#raspberry-pi)
  - [Robot Diagnostics](#robot-diagnostics)
    - [For Just Diagnostics](#for-just-diagnostics)
    - [For AI + Diagnostics](#for-ai--diagnostics)
  - [Robot Auto Test](#robot-auto-test)
- [On Robot Commands](#on-robot-commands)
  - [Systemd Services](#systemd-services)
  - [Debugging Uart](#debugging-uart)

<!--TOC-->

# Common Debugging Steps
```mermaid
---
title: Robot Debugging Steps
---
flowchart TD
    ssh("Can you SSH into the robot? 
        `ssh robot@192.168.5.20RobotID` OR `ssh robot@robot_name.local`
        e.g. `ssh robot@192.168.5.203` or `ssh robot@robert.local`
        for a robot called robert with robot id 3")
    ssh ---> |Yes| tloop_status
    ssh --> |No - Second Try| monitor("Connect Pi to an external monitor and check wifi connection or SSH using an ethernet cable")
    ssh --> |No - First Try| restart(Restart robot)
    restart --> ssh

    diagnostics("`Run Diagnostics while connected to '**tbots**' wifi`") --> robot_view
    robot_view(Robot is shown as connected in 'Robot View' widget?) --> |Yes| check_motors(All motors move?)
    style diagnostics stroke:#f66,stroke-width:2px,stroke-dasharray: 5 5

    check_motors -->|Yes| field_test(Running AI?)
    field_test -->|No| done(Done)
    style done stroke:#30fa02,stroke-width:2px,stroke-dasharray: 5 5
    field_test -->|Yes| field_test_moves(Does robot move during field test?)
    field_test_moves --> |No| check_shell("`Check that the correct shell is placed on the robot`")
    check_shell
    field_test_moves --> |Yes| done

    robot_view --> |No| ssh
    check_motors -->|No| rip
    rip(Check with a lead)
                  
    subgraph ssh_graph [Commands running on the robot]
    tloop_status(Check if Thunderloop is running? 
                               `service thunderloop status`)
    tloop_status --> |Inactive| tloop_restart(Restart Thunderloop service
                                              `service thunderloop restart`)
    tloop_status --> |Running| tloop_logs(Check Thunderloop logs for errors
                                          `journalctl -fu thunderloop -n 300`)
    tloop_restart --> tloop_status
    end
```


# Off Robot Commands

## Wifi Disclaimer

To use most of these commands you will either need to be on the tbots wifi network (no internet access) or on a WiFi with internet access (`ubcvisitor`) and connected to the robot through ethernet tethering. Note that ethernet tethering doesn't work on `ubcsecure` or `eduroam`.

On the tbots network:
- Raspberry Pi robots will have an IP address of `192.168.6.20<robot_id>` so for robot id `1` the IP is `192.168.6.201`. Occasionally, these robots may have an IP address of `192.168.5.20<robot_id>`.

If you are not using the tbots network you will need to use a network utility (`tshark`, `wireshark`, `arp`) to determine the IP address.

## Miscellaneous Ansible Tasks & Options

Individual miscellaneous tasks (ex reboot, shutdown, rtt test) can be run through the `misc.yml` playbook by specifying the corresponding tag.

To view a list of supported arguments, run:

```bash
bazel run //software/embedded/ansible:run_ansible --platforms=//toolchains/cc:robot -h
```

If desired, the `-ho`, `--hosts` argument can be replaced with `-p`, `--port`, defining a port to listen to for Announcements from hosts.

The `--tags` argument can be used to specify which actions to perform and on which services.

## Flashing the robot's compute module

This will stop the current Systemd services, replace and restart them. Binaries that are used for Systemd services are located in a folder in `home/robot` (the default directory) called `thunderbots_binaries`.

<b>To build this for the first time you will need to run this with internet access. Then run it again on the tbots network</b>

<b>This will trigger motor calibration meaning the wheels may spin. Please elevate the robot so the wheels are not touching the ground for proper calibration.</b>

```bash
bazel run //software/embedded/ansible:run_ansible --platforms=//toolchains/cc:robot -- --playbook deploy_robot_software.yml --hosts <robot_ip> --ssh_pass <robot_password>
```

* <robot_ip> is the IP address of the robot
* <robot_password> is the password of the `robot` user account

You could also use the `tbots.py` script to flash robot software

`./tbots.py run run_ansible -f <robot_ids> -pwd <robot_password>` (Note that this uses robot IDs rather than full robot IP addresses)
* <robot_ids> is a list of robot IDs to flash
* <robot_password> is the password of the `robot` user account

Example: Flashing robots 1, 4, and 7 that have a Raspberry Pi

```bash
./tbots.py run run_ansible -pl PI -f 1 4 7 -pwd <robot_password>
```

## Flashing the powerboard

This will flash powerloop, the current firmware in `software/power/`, onto the power board. It will prompt the user into setting the powerboard into bootloader mode by holding the boot button (left if looking from the back of the robot) and pressing the reset button (right if looking from the back of the robot), then releasing the reset button first, then the boot button. Once the board is flashed, pressing the reset button after to use the new firmware.

Looking from the back of the robot the reset and boot buttons are on right side of the battery holder on the lowest board with the reset being on the left and the boot on the right. <b>Warning it may kick/chip when pressed.</b>

```bash
bazel run //software/embedded/ansible:run_ansible --platforms=//toolchains/cc:robot -- --playbook deploy_powerboard.yml --hosts <robot_ip> --ssh_pass <robot_password>
```

## Setting up the embedded host

This section refers to setting up the computer on the robot for the first time. We need to setup dependencies, drivers and necessary configurations.

<b>Setting up the robot for the first time requires internet access</b>

### Raspberry Pi

```bash
bazel run //software/embedded/ansible:run_ansible --platforms=//toolchains/cc:robot -- --playbook setup_pi.yml --hosts <robot_ip> --ssh_pass <robot_password>
```

## Robot Diagnostics

Robot Diagnostics allow users to input various commands to the robots. It can be used to move the robot in the x, y and theta direction as well as kick, chip, autokick or autochip. It can also be used to spin the dribbler. 

<b>If multiple people are using robot diagnostics at the same time on the same network please make sure each person only connects to the robots they are testing via the checkboxes</b>

### For Just Diagnostics

From Software/src

```bash
./tbots.py run thunderscope --run_diagnostics --interface <network_interface>
```

### For AI + Diagnostics

From Software/src

```bash
./tbots.py run thunderscope --run_blue --run_diagnostics --interface <network_interface>
```

network_interface can be found with `ifconfig` commonly `wlp59s0` for wifi.

## Robot Auto Test
Runs the robot auto test fixture on a robot through Ansible, which tests the motor board and power board SPI and UART transfer respectively.

From Software/src:

```bash
bazel run //software/embedded/ansible:run_ansible --platforms=//toolchains/cc:robot -- --playbook robot_auto_test_playbook.yml --hosts <robot_name> --ssh_pass <robot_password>
```

* replace the \<robot_ip\> with the actual ip address of the Raspberry Pi for the ssh connection.
* replace the <robot_password> with the actual password for the Raspberry Pi for the ssh connection.

# On Robot Commands

`ssh robot@<robot_ip>` can be used to connect to a robot.

## Systemd Services

Status shows whether the service is running and some recent logs. More logs can be found using `journalctl` shown below. More control can be achieved with `systemctl`. Currently, the only valid `<service_name>` is `thunderloop`.  

```bash
service <service_name> status
```

Change whether the service is running or restart it. Valid `<run_command>` are `stop`, `start` and `restart`.

```bash
service <service_name> <run_command>
```

To view the full logs in vi/vim:  

```bash
journalctl <service_name>
```

`Shift + g` to jump to bottom

`Shift + z` twice to exit

To follow the recent outputs to the log:

```bash
journalctl -fu <service_name>
```

## Debugging Uart

`screen` or [GNU screen](https://www.gnu.org/software/screen/) can act as serial monitor displaying what the Raspberry Pi is receiving and allowing us to send characters through the terminal. 

For uart we use serial port: `/dev/ttyTHS1`  
For usb we use serial port: `/dev/ttyUSB0`  

The baudrate of powerloop is `460800` other programs will have different baudrates commonly `115200`. If the baudrate is incorrect you will see foreign characters.

Usage:

```bash
screen <serial_port> <baudrate>
```

Exiting:

`Ctrl-a` -> `k` -> `y` -> `enter`

Debugging:

If the serial_port is busy, screen will not launch and instead says `screen is terminating`. This may be because the port is in use either by thunderloop or another process running screen.

Powerloop uart communication is encoded so you can't read it from screen and will appear as a mix of foreign characters

Pressing the reset button once will send a status msg over its connected port. This is useful for sanity checking.
