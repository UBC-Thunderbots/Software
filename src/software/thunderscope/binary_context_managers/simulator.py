import os
import logging
import time

from subprocess import Popen
from software.python_bindings import *
from proto.import_all_protos import *
from software.py_constants import *
from software.thunderscope.proto_unix_io import ProtoUnixIO
from extlibs.er_force_sim.src.protobuf.world_pb2 import SimulatorState
from software.thunderscope.binary_context_managers.util import *


class Simulator(object):

    """ Simulator Context Manager """

    def __init__(
        self,
        simulator_runtime_dir: os.PathLike = None,
        debug_simulator: bool = False,
        enable_realism: bool = False,
    ) -> None:
        """Run Simulator

        NOTE: If any of the runtime directories are None, the corresponding binary
              will not be launched.

        :param simulator_runtime_dir: The directory to run the simulator in
        :param debug_simulator: Whether to run the simulator in debug mode
        :param enable_realism: a argument (--enable_realism) that is going to be passed to er_force_simulator_main binary

        """
        self.simulator_runtime_dir = simulator_runtime_dir
        self.debug_simulator = debug_simulator
        self.er_force_simulator_proc = None
        self.enable_realism = enable_realism

    def __enter__(self) -> "self":
        """Enter the simulator context manager. 

        If the debug mode is enabled then the binary is _not_ run and the
        command to debug under gdb is printed.

        :return: simulator context managed instance

        """
        # Setup unix socket directory
        try:
            os.makedirs(self.simulator_runtime_dir)
        except:
            pass

        simulator_command = "software/er_force_simulator_main --runtime_dir={}".format(
            self.simulator_runtime_dir
        )

        if self.enable_realism:
            simulator_command += " --enable_realism"

        if self.debug_simulator:

            # We don't want to check the exact command because this binary could
            # be debugged from clion or somewhere other than gdb
            if not is_cmd_running(
                [
                    "er_force_simulator_main",
                    "--runtime_dir={}".format(self.simulator_runtime_dir),
                ]
            ):
                logging.info(
                    (
                        f"""
Debugging Simulator ==============
1. Build the simulator in debug mode:

./tbots.py -d build er_force_simulator_main

2. Run the following binary from src to debug the simulator:

gdb --args bazel-bin/{simulator_command}

3. Rerun this binary once the gdb instance is setup
"""
                    )
                )

                # Wait for the user to exit and restart the binary
                while True:
                    time.sleep(1)
        else:
            self.er_force_simulator_proc = Popen(simulator_command.split(" "))

        return self

    def __exit__(self, type, value, traceback) -> None:
        """Exit the full_system context manager.

        :param type: The type of exception that was raised
        :param value: The exception that was raised
        :param traceback: The traceback of the exception

        """
        if self.er_force_simulator_proc:
            self.er_force_simulator_proc.kill()
            self.er_force_simulator_proc.wait()

    def setup_proto_unix_io(
        self,
        simulator_proto_unix_io: ProtoUnixIO,
        blue_full_system_proto_unix_io: ProtoUnixIO,
        yellow_full_system_proto_unix_io: ProtoUnixIO,
    ) -> None:

        """Setup the proto unix io for the simulator

        :param simulator_proto_unix_io: The proto unix io of the simulator.
        :param blue_full_system_proto_unix_io: The proto unix io of the blue full system.
        :param yellow_full_system_proto_unix_io: The proto unix io of the yellow full system.

        """

        # inputs to er_force_simulator_main
        for arg in [
            (SIMULATION_TICK_PATH, SimulatorTick),
            (WORLD_STATE_PATH, WorldState),
        ]:
            simulator_proto_unix_io.attach_unix_sender(self.simulator_runtime_dir, *arg)

        simulator_proto_unix_io.attach_unix_receiver(
            self.simulator_runtime_dir,
            WORLD_STATE_RECEIVED_TRIGGER_PATH,
            WorldStateReceivedTrigger,
        )

        # setup blue full system unix io
        for arg in [
            (BLUE_WORLD_PATH, World),
            (BLUE_PRIMITIVE_SET, PrimitiveSet),
        ]:
            blue_full_system_proto_unix_io.attach_unix_sender(
                self.simulator_runtime_dir, *arg
            )

        for arg in [
            (BLUE_SSL_WRAPPER_PATH, SSL_WrapperPacket),
            (BLUE_ROBOT_STATUS_PATH, RobotStatus),
            (SIMULATOR_STATE_PATH, SimulatorState),
        ] + [
            # TODO (#2655): Add/Remove HRVO layers dynamically based on the HRVOVisualization proto messages
            (BLUE_HRVO_PATH, HRVOVisualization, True)
            for _ in range(MAX_ROBOT_IDS_PER_SIDE)
        ]:
            blue_full_system_proto_unix_io.attach_unix_receiver(
                self.simulator_runtime_dir, *arg
            )

        # setup yellow full system unix io
        for arg in [
            (YELLOW_WORLD_PATH, World),
            (YELLOW_PRIMITIVE_SET, PrimitiveSet),
        ]:
            yellow_full_system_proto_unix_io.attach_unix_sender(
                self.simulator_runtime_dir, *arg
            )

        for arg in [
            (YELLOW_SSL_WRAPPER_PATH, SSL_WrapperPacket),
            (YELLOW_ROBOT_STATUS_PATH, RobotStatus),
        ] + [
            # TODO (#2655): Add/Remove HRVO layers dynamically based on the HRVOVisualization proto messages
            (YELLOW_HRVO_PATH, HRVOVisualization, True)
            for _ in range(MAX_ROBOT_IDS_PER_SIDE)
        ]:
            yellow_full_system_proto_unix_io.attach_unix_receiver(
                self.simulator_runtime_dir, *arg
            )
