from ansible import context
from ansible.cli import CLI
from ansible.module_utils.common.collections import ImmutableDict
from ansible.executor.playbook_executor import PlaybookExecutor
from ansible.parsing.dataloader import DataLoader
from ansible.inventory.manager import InventoryManager
from ansible.vars.manager import VariableManager
import os
import argparse

HOST_GROUP = "THUNDERBOTS_HOSTS"


def ansible_runner(playbook, hosts, tags):

    loader = DataLoader()

    context.CLIARGS = ImmutableDict(tags={}, listtags=False, listtasks=False, listhosts=False, syntax=False, connection='ssh',
                                module_path=None, forks=16, remote_user='dev', private_key_file=None,
                                ssh_common_args=None, ssh_extra_args=None, sftp_extra_args=None, scp_extra_args=None, become=True,
                                verbosity=True, check=False, start_at_task=None)

    inventory = InventoryManager(loader=loader, sources=())
    inventory.add_group(HOST_GROUP)

    for host in hosts:
        inventory.add_host(host, HOST_GROUP)

    variable_manager = VariableManager(loader=loader, inventory=inventory, version_info=CLI.version_info(gitinfo=False))

    pbex = PlaybookExecutor(playbooks=[os.path.dirname(__file__)+'/playbooks/'+playbook], inventory=inventory, variable_manager=variable_manager, loader=loader, passwords={})

    results = pbex.run()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--playbook", required=True, help="The YAML playbook to run")
    ap.add_argument("--hosts", nargs="*", required=False, help="space seperated list of hosts to run on", default=[])
    ap.add_argument("--tags", nargs="*", required=False, help="space seperated list of hosts to run on", default=[])
    args = vars(ap.parse_args())
    ansible_runner(playbook=args['playbook'], ip=args['hosts'], tags=['tags'])


if __name__ == "__main__":
    main()