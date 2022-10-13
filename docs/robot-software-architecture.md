# Robot Software Architecture

![Robot Software Diagram](images/robot_software_diagram.svg)

## Ansible & Systemd

### How Ansible works

Ansible automation jobs, called “Playbooks”, are written in Yaml. Automation jobs contain a series of tasks (ex move a file, run this script, output this command) and logic dictating dependencies between tasks. When playbooks are run, Ansible establishes an SSH connection between the user's computer and target Nanos, allowing it to run the tasks in the playbook. Output from each task, and any other requested output, is displayed on the console 

Example command: `bazel run //software/jetson_nano/ansible:run_ansible --cpu=jetson_nano -- -pb deploy_nano.yml --hosts <192.168.0.robot_id> --ssh_pass thunderbots`

More commands available [here](useful-robot-commands.md)

### Systemd

Systemd allows us to have services which start as soon as we boot the robot, will automatically restart and are individually controllable. All services have the file {service}.service, which controls the configuration of that service. Currently we have a service for thunderloop, announcements and display

## Redis

Redis is an in-memory key-value store. This allows us to share state between processes as well as modify values dynamically through the provided cli. It also persists between boots

## Thunderloop

Thunderloop is software that runs in a loop. It continuously polls services(unrelated from systemd) sending relevant control proto(PowerControl, MotorControl) and receiving back status proto. Currently we have a Network, Power and Motor service. Thunderloop also receives World and Primitive Proto from Ai and sends back Robot Status.

## Announcements
 
Broadcasts packets for what robot ids are connected to wifi and what IPs they have. 