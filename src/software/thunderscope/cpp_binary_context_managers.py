import os
from proto.import_all_protos import *
from subprocess import Popen
from software.networking import threaded_unix_sender, networking
from software.py_constants import *
from extlibs.er_force_sim.src.protobuf.world_pb2 import (
    SimulatorState,
    SimBall,
    SimRobot,
)


class FullSystem(object):

    """ Full System Binary Context Manager """

    def __init__(
        self,
        fullsystem_runtime_dir=None,
        debug_fullsystem=False,
        friendly_colour_yellow=False,
    ):
        """Run FullSystem

        NOTE: If any of the runtime directories are None, the corresponding binary
              will not be launched.

        :param fullsystem_runtime_dir: The directory to run the blue fullsystem in
        :param debug_fullsystem: Whether to run the fullsystem in debug mode

        """
        self.fullsystem_runtime_dir = fullsystem_runtime_dir
        self.debug_fullsystem = debug_fullsystem
        self.friendly_colour_yellow = friendly_colour_yellow

    def __enter__(self):
        """Enter the fullsystem context manager. 

        If the debug mode is enabled then the binary is _not_ run and the
        command to debug under gdb is printed.

        :return: FullSystem instance

        """
        # Setup unix socket directory
        try:
            os.mkdir(self.fullsystem_runtime_dir)
        except:
            pass

        full_system = "software/unix_full_system --runtime_dir={} {}".format(
            self.fullsystem_runtime_dir,
            "--friendly_colour_yellow" if self.friendly_colour_yellow else "",
        )

        if self.debug_fullsystem:
            print("============ Debugging Full System ==============")

            print("\n1. Build full system in debug mode")
            print("./tbots.py -d build unix_full_system")

            print("\n2. Run the following binaries from src to debug full system")
            print("gdb --args bazel-bin/{}".format(full_system))

        else:
            self.fullsystem_proc = Popen(full_system.split(" "))

        return self

    def __exit__(self, type, value, traceback):
        """Exit the fullsystem context manager.

        :param type: The type of exception that was raised
        :param value: The exception that was raised
        :param traceback: The traceback of the exception

        """
        self.fullsystem_proc.kill()
        self.fullsystem_proc.wait()

    def setup_proto_unix_io(self, proto_unix_io):
        """Helper to run full system and attach the appropriate unix senders/listeners

        :param proto_unix_io: The unix io to setup for this fullsystem instance

        """

        # Setup LOG(VISUALIZE) handling from full system. We set from_log_visualize
        # to true.
        for arg in [
            (self.fullsystem_runtime_dir, Obstacles, True),
            (self.fullsystem_runtime_dir, PathVisualization, True),
            (self.fullsystem_runtime_dir, PassVisualization, True),
            (self.fullsystem_runtime_dir, NamedValue, True),
            (self.fullsystem_runtime_dir, PlayInfo, True),
        ]:
            proto_unix_io.attach_unix_receiver(*arg)

        proto_unix_io.attach_unix_receiver(
            self.fullsystem_runtime_dir + "/log", RobotLog
        )

        # Inputs to full_system
        for arg in [
            (self.fullsystem_runtime_dir + ROBOT_STATUS_PATH, RobotStatus),
            (self.fullsystem_runtime_dir + SSL_WRAPPER_PATH, SSL_WrapperPacket),
            (self.fullsystem_runtime_dir + SSL_REFEREE_PATH, Referee),
            (self.fullsystem_runtime_dir + SENSOR_PROTO_PATH, SensorProto),
            (
                self.fullsystem_runtime_dir + TACTIC_OVERRIDE_PATH,
                AssignedTacticPlayControlParams,
            ),
        ]:
            proto_unix_io.attach_unix_sender(*arg)

        # Outputs from full_system
        proto_unix_io.attach_unix_receiver(
            self.fullsystem_runtime_dir + WORLD_PATH, World
        )
        proto_unix_io.attach_unix_receiver(
            self.fullsystem_runtime_dir + PRIMITIVE_PATH, PrimitiveSet
        )


class Simulator(object):

    """ Simulator Context Manager """

    def __init__(self, simulator_runtime_dir=None, debug_simulator=False):
        """Run Simulator

        NOTE: If any of the runtime directories are None, the corresponding binary
              will not be launched.

        :param simulator_runtime_dir: The directory to run the simulator in
        :param debug_simulator: Whether to run the simulator in debug mode

        """
        self.simulator_runtime_dir = simulator_runtime_dir
        self.debug_simulator = debug_simulator

    def __enter__(self):
        """Enter the simulator context manager. 

        If the debug mode is enabled then the binary is _not_ run and the
        command to debug under gdb is printed.

        :return: simulator instance

        """
        # Setup unix socket directory
        try:
            os.mkdir(self.simulator_runtime_dir)
        except:
            pass

        simulator_command = "software/er_force_simulator_main --runtime_dir={}".format(
            self.simulator_runtime_dir
        )

        if self.debug_simulator:
            print("============== Debugging Simulator ==============")

            print("\n1. Build the simulator in debug mode:")
            print("./tbots.py -d build er_force_simulator_main")

            print("\n2. Run the following binary from src to debug the simulator:")
            print("gdb --args bazel-bin/{}\n".format(simulator_command))

        else:
            self.er_force_simulator_proc = Popen(simulator_command.split(" "))

        return self

    def __exit__(self, type, value, traceback):
        """Exit the fullsystem context manager.

        :param type: The type of exception that was raised
        :param value: The exception that was raised
        :param traceback: The traceback of the exception

        """
        self.er_force_simulator_proc.kill()
        self.er_force_simulator_proc.wait()

    def setup_proto_unix_io(
        self,
        simulator_proto_unix_io,
        blue_full_system_proto_unix_io,
        yellow_full_system_proto_unix_io,
    ):

        """Setup the proto unix io for the simulator

        :param simulator_proto_unix_io: The proto unix io of the simulator.
        :param blue_full_system_proto_unix_io: The proto unix io of the blue full system.
        :param yellow_full_system_proto_unix_io: The proto unix io of the yellow full system.

        """

        # inputs to er_force_simulator_main
        for arg in [
            (self.simulator_runtime_dir + SIMULATION_TICK_PATH, SimulatorTick),
            (self.simulator_runtime_dir + WORLD_STATE_PATH, WorldState),
        ]:
            simulator_proto_unix_io.attach_unix_sender(*arg)

        # setup blue full system unix io
        for arg in [
            (self.simulator_runtime_dir + BLUE_WORLD_PATH, World),
            (self.simulator_runtime_dir + BLUE_PRIMITIVE_SET, PrimitiveSet),
        ]:
            blue_full_system_proto_unix_io.attach_unix_sender(*arg)

        for arg in [
            (self.simulator_runtime_dir + BLUE_SSL_WRAPPER_PATH, SSL_WrapperPacket),
            (self.simulator_runtime_dir + BLUE_ROBOT_STATUS_PATH, RobotStatus),
            (self.simulator_runtime_dir + SIMULATOR_STATE_PATH, SimulatorState),
        ]:
            blue_full_system_proto_unix_io.attach_unix_receiver(*arg)

        # setup yellow full system unix io
        for arg in [
            (self.simulator_runtime_dir + YELLOW_WORLD_PATH, World),
            (self.simulator_runtime_dir + YELLOW_PRIMITIVE_SET, PrimitiveSet),
        ]:
            yellow_full_system_proto_unix_io.attach_unix_sender(*arg)

        for arg in [
            (self.simulator_runtime_dir + YELLOW_SSL_WRAPPER_PATH, SSL_WrapperPacket),
            (self.simulator_runtime_dir + YELLOW_ROBOT_STATUS_PATH, RobotStatus),
        ]:
            yellow_full_system_proto_unix_io.attach_unix_receiver(*arg)


class Gamecontroller(object):

    """ Gamecontroller Context Manager """

    def __enter__(self):
        """Enter the gamecontroller context manager. 

        :return: gamecontroller instance

        """
        self.gamecontroller_proc = Popen(["/opt/tbotspython/gamecontroller"])

        return self

    def __exit__(self, type, value, traceback):
        """Exit the gamecontroller context manager.

        :param type: The type of exception that was raised
        :param value: The exception that was raised
        :param traceback: The traceback of the exception

        """
        self.gamecontroller_proc.kill()
        self.gamecontroller_proc.wait()

    def setup_proto_unix_io(
        self, blue_full_system_proto_unix_io, yellow_full_system_proto_unix_io
    ):
        """Setup gamecontroller io

        :param blue_full_system_proto_unix_io: The proto unix io of the blue full system.
        :param yellow_full_system_proto_unix_io: The proto unix io of the yellow full system.

        """

        def __send_referee_command(data):
            blue_full_system_proto_unix_io.send_proto(Referee, data)
            yellow_full_system_proto_unix_io.send_proto(Referee, data)

        # TODO pull the constants out into a file
        self.receive_referee_command = networking.SSLRefereeProtoListener(
            "224.5.23.1", 10003, __send_referee_command, True,
        )
