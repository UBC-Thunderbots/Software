# Robot Software Architecture

![Robot Software Diagram](images/robot_software_diagram.svg)

## Ansible & Systemd

### How Ansible works

Ansible automation jobs, called “Playbooks”, are written in Yaml. Automation jobs contain a series of tasks (ex move a file, run this script, output this command) and logic dictating dependencies between tasks. When playbooks are run, Ansible establishes an SSH connection between the user's computer and each Nano, allowing it to run the tasks in the playbook. Output from each task, and any other requested output, is displayed on the console 

Example command: `bazel`

### Systemd

All services have the file {service}.service, which controls the configuration of that service. All services except for thunderbots have a watcher@{service}.path and watcher@{service}.service files. These control restarting the program when receiving a new binary/script. The overall file hierarchy looks something like this, where service can be one of announcements, ui, tftp, or redis: 

/etc/systemd/system
Thunderbots
Thunderbots.service
{service}
{service}.service
watcher@{service}.path
watcher@{service}.service


## Redis

## Thunderloop

## Announcements
 
Broadcasts packets for what robots are connected to wifi and what IPs they have