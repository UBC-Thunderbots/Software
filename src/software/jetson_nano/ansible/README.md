
# Ansible & Systemd User Guide

Ansible allows us to run actions on multiple robots at once. Actions are communicated through YAML files called playbooks.
For a more detailed look at how Ansible works, see the RFC (https://docs.google.com/document/d/1hN3Us2Vjr8z6ihqUVp_3L7rrjKc-EZ-l2hZJc31gNOc/edit)

Systemd is a low level service that manages the automattic running of processes on a single robot. We also use it enable remote flashing. To learn more about how it works, see the RFC (https://docs.google.com/document/d/1hN3Us2Vjr8z6ihqUVp_3L7rrjKc-EZ-l2hZJc31gNOc/edit)



### Ansible Setup
Ansible is integrated in our Bazel build environment. All playbooks must be added to the `src/jetson_nano/ansible/playbooks` directory. 
Before running any command, ensure that: 

1) all hosts (robots/jetsons) and the control node (machine that is running ansible) are on the same network. 

2) all hosts have run `setup_nano.sh` script at least once before. This can also be done through Ansible via the command 
`bazel run :run_ansible --cpu=jetson_nano -- -pb setup_nano.yml -ho {ip addresses of hosts} -pwd {robot_ssh_password} `

### How To Run an Ansible Playbook

To run a playbook called `playbook.yml`, use the command 
``bazel run :run_ansible --cpu=jetson_nano -- -pb playbook.yml -ho [list of ip addrs] -pwd {ssh_pass}`` 

If desired, the `-ho` argument can be replaced with `-p`, defining a port to listen to for Announcements from hosts. 

Different arguments might need to be added to support different scenarios (ex tags, list of hosts). To view a list of supported arguments, run 
``bazel run :run_ansible --cpu=jetson_nano -- -h`` 


### Remote Flashing

After Ansible is setup, remote flashing, done through Systemd, can be setup by running the `setup_systemd.yml` playbook. 

Running the `setup_systemd.yml` playbook will copy all relevant thunderbots binaries into the Jetson Nano and reboot the nano. After rebooting, the binaries will automatically be run. Upon new binaries being pasted into the Nano (in the right directory), the Nano will restart the intended process. 

The `sync_binaries.yml` playbook can be used to push new binaries to hosts, while the "--tags" argument can be used to specify which binaries to push. See `sync_binaries.yml` for tag names. 

