from ansible import context
from ansible.cli import CLI
from ansible.module_utils.common.collections import ImmutableDict
from ansible.executor.playbook_executor import PlaybookExecutor
from ansible.parsing.dataloader import DataLoader
from ansible.inventory.manager import InventoryManager
from ansible.vars.manager import VariableManager
from software.embedded.constants.py_constants import RobotPlatform, NetworkConstants

import sys
import os
import argparse
import subprocess

HOST_GROUP = "THUNDERBOTS_HOSTS"
HOST_USERNAME = "robot"
DEFAULT_SSH_CONNECTION_TIMEOUT = 60
MAX_NUM_ROBOTS = 8


def ansible_runner(playbook: str, options: dict = {}) -> int:
    """Run an Ansible playbook.

    Ansible is used to remotely run scripts on multiple robots (hosts) at once.
    Documentation on the Ansible Python API can be found here:
    https://docs.ansible.com/ansible/latest/dev_guide/developing_api.html

    :param playbook: the playbook to run
    :param options: the options to pass to the playbook executor
    :return: the exit code from the playbook executor
    """
    loader = DataLoader()

    # parse options
    vars = set(options.get("extra_vars", []))
    tags = set(options.get("tags", {}))
    skip_tags = set(options.get("skip_tags", {}))
    ssh_pass = options.get("ssh_pass", "")
    timeout = options.get("timeout", DEFAULT_SSH_CONNECTION_TIMEOUT)

    hosts = set(options.get("hosts", []))
    host_aliases = hosts.copy()

    if not hosts:
        # Spawn multiple processes to ping different robots
        ping_processes = {}

        for robot_id in range(MAX_NUM_ROBOTS):
            for platform in RobotPlatform:
                ip = str(NetworkConstants.get_ip_address(robot_id, platform))

                # Ping 3 times waiting 1s for timeout
                command = f"ping -w 1 -c 3 {ip}"
                ping_processes[(robot_id, ip)] = subprocess.Popen(
                    command.split(), stdout=subprocess.DEVNULL
                )

        while ping_processes:
            for (robot_id, ip), proc in ping_processes.items():
                # Check status of ping processes
                if proc.poll() is not None:
                    del ping_processes[(robot_id, ip)]
                    if proc.returncode == 0:
                        hosts.add(ip)
                        host_aliases.add(robot_id)
                    break

    num_forks = len(hosts)

    # bunch of arguments that Ansible accepts
    context.CLIARGS = ImmutableDict(
        tags=tags,
        listtags=False,
        listtasks=False,
        listhosts=False,
        syntax=False,
        connection="ssh",
        module_path=None,
        forks=num_forks,
        remote_user=HOST_USERNAME,
        private_key_file=None,
        ssh_common_args="-o StrictHostKeyChecking=no",
        ssh_extra_args=None,
        sftp_extra_args=None,
        scp_extra_args=None,
        become=False,
        verbosity=True,
        check=False,
        start_at_task=None,
        extra_vars=vars,
        skip_tags=skip_tags,
        timeout=timeout,
    )

    # for ansible, an inventory represents our fleet of robots. Each host in the inventory belongs to a group.
    inventory = InventoryManager(loader=loader, sources=())

    # a variable manager maintains variables for each individual host. Group or global variables can also be added
    variable_manager = VariableManager(
        loader=loader, inventory=inventory, version_info=CLI.version_info(gitinfo=False)
    )

    group = inventory.add_group(HOST_GROUP)

    # adding hosts and their aliases (robot IDs if available) to the inventory
    for host, alias in zip(hosts, host_aliases):
        inventory.add_host(host, group)
        variable_manager.set_host_variable(host, "inventory_hostname", alias)

    # playbook executor controls running the playbook
    pbex = PlaybookExecutor(
        playbooks=[os.path.dirname(__file__) + "/playbooks/" + playbook],
        inventory=inventory,
        variable_manager=variable_manager,
        loader=loader,
        passwords={"conn_pass": ssh_pass, "become_pass": ssh_pass},
    )

    return pbex.run()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--playbook",
        "-pb",
        required=True,
        help="Ansible playbook to run (must include the .yml extension)",
    )
    ap.add_argument(
        "--ssh_pass", "-pwd", required=False, help="Password to ssh into hosts"
    )
    ap.add_argument(
        "--hosts",
        "-ho",
        nargs="*",
        required=False,
        help="space separated list of hosts to run on",
        default=[],
    )
    ap.add_argument(
        "--tags",
        "-t",
        nargs="*",
        required=False,
        help="space separated list of tags to run",
        default=[],
    )
    ap.add_argument(
        "--skip_tags",
        "-st",
        nargs="*",
        required=False,
        help="space separated list of tags to skip",
        default=[],
    )
    ap.add_argument(
        "--extra_vars",
        "-e",
        nargs="*",
        required=False,
        help="space separated list of variables to set in the form key=value",
        default=[],
    )
    ap.add_argument(
        "--timeout",
        "-to",
        required=False,
        help="SSH connection timeout in seconds",
        default=DEFAULT_SSH_CONNECTION_TIMEOUT,
    )

    args = vars(ap.parse_args())

    ansible_result = ansible_runner(playbook=args["playbook"], options=args)

    sys.exit(ansible_result)
