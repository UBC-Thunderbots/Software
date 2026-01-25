import logging
import psutil
from collections.abc import Iterator
from software.python_bindings import *
from proto.import_all_protos import *
from software.py_constants import *


def _find_cmd(command: list[str]) -> Iterator[psutil.Process]:
    """Iterate over running processes that were launched with the given command.

    :param command: Command to match against. List of strings.
    :yield: Processes that match the given command.
    """
    for proc in psutil.process_iter(['cmdline']):
        try:
            cmdline = proc.info['cmdline']
            if not cmdline:
                continue

            command_found = " ".join(cmdline)
            for string in command:
                if string not in command_found:
                    break
            else:
                yield proc
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass

def is_cmd_running(command: list[str]) -> bool:
    """Check if there is any running process that was launched with the given command.

    :param command: Command to match against. List of strings.
    :return: whether there is a running process that was launched with the given command
    """
    for _ in _find_cmd(command):
        return True
    return False

def kill_cmd_if_running(command: list[str]) -> None:
    """Kill (not terminate) all running processes that was launched with the given command.

    :param command: Command to match against. List of strings.
    """
    for proc in _find_cmd(command):
        try:
            logging.info(f"Killing existing {command[0]} process PID={proc.pid}")
            proc.kill()
            proc.wait(timeout = 3)
        except (psutil.NoSuchProcess, psutil.TimeoutExpired):
            pass
