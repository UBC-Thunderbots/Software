import psutil as util
from software.python_bindings import *
from proto.import_all_protos import *
from software.py_constants import *


def is_cmd_running(command):
    """Check if there is any running process that was launched
    with the given command.

    :param command: Command that was used to launch the process. List of strings.

    """
    for proc in util.process_iter():
        try:
            for string in command:
                if string not in "".join(proc.cmdline()):
                    break
            else:
                return True
        except (util.NoSuchProcess, util.AccessDenied, util.ZombieProcess):
            pass

    return False
