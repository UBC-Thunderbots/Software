import psutil
import google.protobuf.internal.encoder as encoder
import google.protobuf.internal.decoder as decoder

from software.python_bindings import *
from proto.import_all_protos import *
from software.py_constants import *
from extlibs.er_force_sim.src.protobuf.world_pb2 import (
    SimulatorState,
    SimBall,
    SimRobot,
)


def is_cmd_running(command):
    """Check if there is any running process that was launched
    with the given command.

    :param command: Command that was used to launch the process. List of strings.

    """
    for proc in psutil.process_iter():
        try:
            for string in command:
                if string not in "".join(proc.cmdline()):
                    break
            else:
                return True
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass

    return False
