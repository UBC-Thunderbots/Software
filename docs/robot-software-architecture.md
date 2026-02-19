@page robot_software_architecture Robot Software Architecture
[TOC]

# Robot Software Diagram

![Robot Software Diagram](images/robot_software_diagram.svg)

# Tools

## Ansible

[Ansible](https://www.ansible.com/overview/how-ansible-works) allows us to run actions on multiple robots at once. Actions are communicated through YAML files called playbooks. Playbooks contain a series of tasks (eg. move a file, run this script, output this command) and logic dictating dependencies between tasks. When playbooks are run, Ansible establishes an SSH connection between the user's computer and robot, allowing it to run the tasks in the playbook. Output from each task, and any other requested output, is displayed on the console

For a more detailed look at how Ansible works, [see the RFC](https://docs.google.com/document/d/1hN3Us2Vjr8z6ihqUVp_3L7rrjKc-EZ-l2hZJc31gNOc/edit)

Example command: `bazel run //software/embedded/ansible:run_ansible --platforms=//toolchains/cc:robot --//software/embedded:host_platform=<platform> -- --playbook deploy_robot_software.yml --hosts <robot_ip> --ssh_pass <robot_password>`
* <platform>: `PI` or `NANO` depending on the computer on the robot
* <robot_ip>: IP address of the robot
* <robot_password>: Password of the robot

More commands available [here](useful-robot-commands.md#off-robot-commands)

## Systemd

[Systemd](https://www.freedesktop.org/wiki/Software/systemd/) allows us to have services which start as soon as we boot the robot, will automatically restart and are individually controllable. All services have the file {service}.service, which controls the configuration of that service. Our core service brought up by systemd is thunderloop. The thunderloop.service file can be seen [here](https://github.com/UBC-Thunderbots/Software/blob/master/src/software/embedded/linux_configs/systemd/thunderloop.service).

## Redis

[Redis](https://redis.io/docs/about/) is an in-memory key-value store. This allows us to share state between processes as well as modify values dynamically through the provided [cli](useful-robot-commands#redis). Values also persists between boots.

# Thunderloop

Thunderloop is software that runs in a loop. It continuously polls services (unrelated from Systemd), sending relevant control protos (`PowerControl`, `MotorControl`) and receiving back status protos from the power and motor boards. Currently we have a [Network](https://github.com/UBC-Thunderbots/Software/blob/master/src/software/embedded/services/network/network.cpp), [Power](https://github.com/UBC-Thunderbots/Software/blob/master/src/software/embedded/services/power.cpp) and [Motor](https://github.com/UBC-Thunderbots/Software/blob/master/src/software/embedded/services/motor.cpp) services. Thunderloop also receives `World` and `PrimitiveSet` protos from AI and sends back `Robot Status` protos.

Motor and Power services both interface with their respective electrical boards over different communication interfaces, namely SPI and UART respectively.
