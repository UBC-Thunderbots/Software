from ansible import context
from ansible.cli import CLI
from ansible.module_utils.common.collections import ImmutableDict
from ansible.executor.playbook_executor import PlaybookExecutor
from ansible.parsing.dataloader import DataLoader
from ansible.inventory.manager import InventoryManager
from ansible.vars.manager import VariableManager
from software.jetson_nano.broadcasts.robot_broadcast_receiver import (
    receive_announcements,
)
import os
import argparse

HOST_GROUP = "THUNDERBOTS_HOSTS"
NANO_USER = "propbot"
ANNOUNCEMENT_LISTEN_DURATION_S = 2

# loads variables, inventory, and play into Ansible API, then runs it
def ansible_runner(playbook: str, options: dict = {}):
    loader = DataLoader()

    print("starting ansible run")

    # parse options
    vars = set(options.get("extra_vars", {}))
    tags = set(options.get("tags", {}))
    skip_tags = set(options.get("skip_tags", {}))
    ssh_pass = options.get("ssh_pass", "")

    hosts = set(options.get("hosts", []))
    host_aliases = hosts

    if not hosts:
        if not options['port']:
            print("Announcement Port not defined, exiting")
            exit()

        announcements = receive_announcements(port=options['port'], duration=ANNOUNCEMENT_LISTEN_DURATION_S)
        hosts = [a.ip_addr for a in announcements]
        host_aliases = [a.robot_id for a in announcements]

    num_forks = len(hosts)

    context.CLIARGS = ImmutableDict(
        tags=tags,
        listtags=False,
        listtasks=False,
        listhosts=False,
        syntax=False,
        connection="ssh",
        module_path=None,
        forks=num_forks,
        remote_user=NANO_USER,
        private_key_file=None,
        ssh_common_args=None,
        ssh_extra_args=None,
        sftp_extra_args=None,
        scp_extra_args=None,
        become=False,
        verbosity=True,
        check=False,
        start_at_task=None,
        extra_vars=vars,
        skip_tags=skip_tags,
    )

    inventory = InventoryManager(loader=loader, sources=())

    variable_manager = VariableManager(
        loader=loader, inventory=inventory, version_info=CLI.version_info(gitinfo=False)
    )

    group = inventory.add_group(HOST_GROUP)

    # adding hosts and their aliases (robot IDs) to the inventory
    for host, alias in zip(hosts, host_aliases):
        inventory.add_host(host, group)
        variable_manager.set_host_variable(host, "inventory_hostname", alias)

    pbex = PlaybookExecutor(
        playbooks=[os.path.dirname(__file__) + "/playbooks/" + playbook],
        inventory=inventory,
        variable_manager=variable_manager,
        loader=loader,
        passwords={"conn_pass": ssh_pass, 'become_pass': ssh_pass},
    )

    pbex.run()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--playbook", "-p", required=True, help="The YAML playbook to run")
    ap.add_argument(
        "--ssh_pass", "-pwd", required=False, help="Password to ssh into hosts"
    )
    ap.add_argument(
        "--hosts",
        "-ho",
        nargs="*",
        required=False,
        help="space separated list of hosts to run on, defaults to using robot announcements",
        default=['192.168.1.78'],
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

    args = vars(ap.parse_args())

    ansible_runner(playbook=args["playbook"], options=args)


if __name__ == "__main__":
    main()
