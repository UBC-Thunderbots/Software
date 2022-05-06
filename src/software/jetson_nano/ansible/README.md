
# Ansible & Systemd User Guide

Ansible allows us to run actions on multiple robots at once. Actions are communicated through YAML files called playbooks.
For a more detailed look at how Ansible works, see the RFC (https://docs.google.com/document/d/1hN3Us2Vjr8z6ihqUVp_3L7rrjKc-EZ-l2hZJc31gNOc/edit)

Systemd is a low level service that manages the automattic running of processes on a single robot. We also use it enable remote flashing. To learn more about how it works, see the RFC (https://docs.google.com/document/d/1hN3Us2Vjr8z6ihqUVp_3L7rrjKc-EZ-l2hZJc31gNOc/edit)



### Ansible Setup
Ansible is integrated in our Bazel build environment. All playbooks must be added to the `src/jetson_nano/ansible/playbooks` directory. 
Before running any command, ensure that: 

1) all hosts (robots/jetsons) and the control node (machine that is running ansible) are on the same network. 

2) all hosts have installed the necessary dependencies. This can be done by running the `setup_nano.sh` script through Ansible via the command 
`bazel run :run_ansible --cpu=jetson_nano -- -pb setup_nano.yml -ho {ip addresses of hosts} -pwd {robot_ssh_password} `

### How To Run an Ansible Playbook

To run a playbook called `playbook.yml`, use the command 
``bazel run :run_ansible --cpu=jetson_nano -- -pb playbook.yml -ho [list of ip addrs] -pwd {ssh_pass}`` 

If desired, the `-ho` argument can be replaced with `-p`, defining a port to listen to for Announcements from hosts. 

Different arguments might need to be added to support different scenarios (ex tags, list of hosts). To view a list of supported arguments, run 
``bazel run :run_ansible --cpu=jetson_nano -- -h`` 


### Remote Flashing

After Ansible is setup, remote flashing, done through Systemd, can be setup by running the `setup_systemd.yml` playbook. Ex: 
``bazel run :run_ansible --cpu=jetson_nano -- -pb setup_systemd.yml -ho [list of ip addrs] -pwd {ssh_pass}`` 

Running the `setup_systemd.yml` playbook will copy all relevant thunderbots binaries into the Jetson Nanos and reboot the nanos. After rebooting, the binaries will automatically be run. 

The `remote_flash.yml` playbook pushes new binaries to hosts and restarts services. The "--tags" argument can be used to specify which binaries to push. If no tags are specified, all binaries will be pushed. Possible tags are: thunderloop, announcement, display, redis. 
For example, to remote flash thunderloop and redis, run the command: 
 ``bazel run :run_ansible --cpu=jetson_nano -- -pb remote_flash.yml -ho [list of ip addrs] -pwd {ssh_pass} -t thunderloop redis`` 

### Miscellaneous Tasks
Individual miscellaneous tasks (ex reboot, rtt test) can be run through the `misc.yml` playbook by specifiying the corresponding tag. 
