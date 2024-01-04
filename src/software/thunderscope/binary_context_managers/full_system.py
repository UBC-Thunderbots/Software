import os
import logging
import time
import threading

from subprocess import Popen
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.python_bindings import *
from proto.import_all_protos import *
from software.py_constants import *
from software.thunderscope.binary_context_managers.util import is_cmd_running


class FullSystem(object):

    """ Full System Binary Context Manager """

    def __init__(
        self,
        full_system_runtime_dir: os.PathLike = None,
        debug_full_system: bool = False,
        friendly_colour_yellow: bool = False,
        should_restart_on_crash: bool = True,
    ) -> None:
        """Run FullSystem

        :param full_system_runtime_dir: The directory to run the blue full_system in
        :param debug_full_system: Whether to run the full_system in debug mode
        :param friendly_color_yellow: a argument passed into the unix_full_system binary (--friendly_colour_yellow)
        :param should_restart_on_crash: whether or not to restart the program after it has been crashed

        """
        self.full_system_runtime_dir = full_system_runtime_dir
        self.debug_full_system = debug_full_system
        self.friendly_colour_yellow = friendly_colour_yellow
        self.full_system_proc = None
        self.should_restart_on_crash = should_restart_on_crash

        self.thread = threading.Thread(target=self.__restart__)

    def __enter__(self) -> "self":
        """Enter the full_system context manager. 

        If the debug mode is enabled then the binary is _not_ run and the
        command to debug under gdb is printed. The  context manager will then
        wait for the binary to be launched before continuing.

        :return: full_system context managed instance

        """
        # Setup unix socket directory
        try:
            os.makedirs(self.full_system_runtime_dir)
        except:
            pass

        self.full_system = "software/unix_full_system --runtime_dir={} {}".format(
            self.full_system_runtime_dir,
            "--friendly_colour_yellow" if self.friendly_colour_yellow else "",
        )

        if self.debug_full_system:

            # We don't want to check the exact command because this binary could
            # be debugged from clion or somewhere other than gdb
            if not is_cmd_running(
                [
                    "unix_full_system",
                    "--runtime_dir={}".format(self.full_system_runtime_dir),
                ]
            ):
                logging.info(
                    (
                        f"""

Debugging Fullsystem ==============
1. Build the full system in debug mode:

./tbots.py -d build unix_full_system

2. Run the following binaries from src to debug full system:

gdb --args bazel-bin/{self.full_system}

3. Rerun this binary once the gdb instance is setup
"""
                    )
                )

                # Wait for the user to exit and restart the binary
                while True:
                    time.sleep(1)

        else:
            self.full_system_proc = Popen(self.full_system.split(" "))
            if self.should_restart_on_crash:
                self.thread.start()

        return self

    def __restart__(self) -> None:
        "Restarts full system."
        while True:
            if not is_cmd_running(
                [
                    "unix_full_system",
                    "--runtime_dir={}".format(self.full_system_runtime_dir),
                ]
            ):
                self.full_system_proc = Popen(self.full_system.split(" "))
                logging.info("FullSystem has restarted.")

            time.sleep(1)

    def __exit__(self, type, value, traceback) -> None:
        """Exit the full_system context manager.

        :param type: The type of exception that was raised
        :param value: The exception that was raised
        :param traceback: The traceback of the exception

        """
        if self.full_system_proc:
            self.full_system_proc.kill()
            self.full_system_proc.wait()

        if self.should_restart_on_crash:
            self.thread.join()

    def setup_proto_unix_io(self, proto_unix_io: ProtoUnixIO) -> None:
        """Helper to run full system and attach the appropriate unix senders/listeners

        :param proto_unix_io: The unix io to setup for this full_system instance

        """

        # Setup LOG(VISUALIZE) handling from full system. We set from_log_visualize
        # to true to decode from base64.
        for proto_class in [
            Obstacles,
            PathVisualization,
            PassVisualization,
            CostVisualization,
            NamedValue,
            PlayInfo,
        ]:
            proto_unix_io.attach_unix_receiver(
                runtime_dir=self.full_system_runtime_dir,
                proto_class=proto_class,
                from_log_visualize=True,
            )

        proto_unix_io.attach_unix_receiver(
            self.full_system_runtime_dir, "/log", RobotLog
        )

        # Outputs from full_system
        proto_unix_io.attach_unix_receiver(
            self.full_system_runtime_dir, WORLD_PATH, World
        )
        proto_unix_io.attach_unix_receiver(
            self.full_system_runtime_dir, PRIMITIVE_PATH, PrimitiveSet
        )

        # Inputs to full_system
        for arg in [
            (ROBOT_STATUS_PATH, RobotStatus),
            (SSL_WRAPPER_PATH, SSL_WrapperPacket),
            (SSL_REFEREE_PATH, Referee),
            (SENSOR_PROTO_PATH, SensorProto),
            (TACTIC_OVERRIDE_PATH, AssignedTacticPlayControlParams,),
            (PLAY_OVERRIDE_PATH, Play),
            (DYNAMIC_PARAMETER_UPDATE_REQUEST_PATH, ThunderbotsConfig,),
        ]:
            proto_unix_io.attach_unix_sender(self.full_system_runtime_dir, *arg)
