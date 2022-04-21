import os
import socket
import logging
import psutil
import time
import google.protobuf.internal.encoder as encoder
import google.protobuf.internal.decoder as decoder

from subprocess import Popen
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


class FullSystem(object):

    """ Full System Binary Context Manager """

    DEBUG_MODE_POLL_INTERVAL_S = 0.1

    def __init__(
        self,
        fullsystem_runtime_dir=None,
        debug_fullsystem=False,
        friendly_colour_yellow=False,
    ):
        """Run FullSystem

        :param fullsystem_runtime_dir: The directory to run the blue fullsystem in
        :param debug_fullsystem: Whether to run the fullsystem in debug mode

        """
        self.fullsystem_runtime_dir = fullsystem_runtime_dir
        self.debug_fullsystem = debug_fullsystem
        self.friendly_colour_yellow = friendly_colour_yellow
        self.fullsystem_proc = None

    def __enter__(self):
        """Enter the fullsystem context manager. 

        If the debug mode is enabled then the binary is _not_ run and the
        command to debug under gdb is printed. The  context manager will then
        wait for the binary to be launched before continuing.

        :return: fullsystem context managed instance

        """
        # Setup unix socket directory
        try:
            os.makedirs(self.fullsystem_runtime_dir)
        except:
            pass

        full_system = "software/unix_full_system --runtime_dir={} {}".format(
            self.fullsystem_runtime_dir,
            "--friendly_colour_yellow" if self.friendly_colour_yellow else "",
        )

        if self.debug_fullsystem:
            logging.info(
                (
                    "\n\nDebugging Fullsystem ==============\n\n"
                    "1. Build the full system in debug mode:\n"
                    "./tbots.py -d build unix_full_system\n\n"
                    "2. Run the following binaries from src to debug full system:\n"
                    f"gdb --args bazel-bin/{full_system}\n"
                    "3. Rerun this binary once the gdb instance is setup\n"
                )
            )

            # We don't want to check the exact command because this binary could
            # be debugged from clion or somewhere other than gdb
            while not is_cmd_running(
                [
                    "unix_full_system",
                    "--runtime_dir={}".format(self.fullsystem_runtime_dir),
                ]
            ):
                time.sleep(Simulator.DEBUG_MODE_POLL_INTERVAL_S)

        else:
            self.fullsystem_proc = Popen(full_system.split(" "))

        return self

    def __exit__(self, type, value, traceback):
        """Exit the fullsystem context manager.

        :param type: The type of exception that was raised
        :param value: The exception that was raised
        :param traceback: The traceback of the exception

        """
        if self.fullsystem_proc:
            self.fullsystem_proc.kill()
            self.fullsystem_proc.wait()

    def setup_proto_unix_io(self, proto_unix_io):
        """Helper to run full system and attach the appropriate unix senders/listeners

        :param proto_unix_io: The unix io to setup for this fullsystem instance

        """

        # Setup LOG(VISUALIZE) handling from full system. We set from_log_visualize
        # to true to decode from base64.
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

    DEBUG_MODE_POLL_INTERVAL_S = 0.1

    def __init__(self, simulator_runtime_dir=None, debug_simulator=False):
        """Run Simulator

        NOTE: If any of the runtime directories are None, the corresponding binary
              will not be launched.

        :param simulator_runtime_dir: The directory to run the simulator in
        :param debug_simulator: Whether to run the simulator in debug mode

        """
        self.simulator_runtime_dir = simulator_runtime_dir
        self.debug_simulator = debug_simulator
        self.er_force_simulator_proc = None

    def __enter__(self):
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

        if self.debug_simulator:
            logging.info(
                (
                    "\n\nDebugging Simulator ==============\n\n"
                    "1. Build the simulator in debug mode:\n"
                    "./tbots.py -d build er_force_simulator_main\n\n"
                    "2. Run the following binary from src to debug the simulator:\n"
                    f"gdb --args bazel-bin/{simulator_command}\n"
                    "3. Rerun this binary once the gdb instance is setup\n"
                )
            )

            # We don't want to check the exact command because this binary could
            # be debugged from clion or somewhere other than gdb
            while not is_cmd_running(
                [
                    "er_force_simulator_main",
                    "--runtime_dir={}".format(self.simulator_runtime_dir),
                ]
            ):
                time.sleep(Simulator.DEBUG_MODE_POLL_INTERVAL_S)

        else:
            self.er_force_simulator_proc = Popen(simulator_command.split(" "))

        return self

    def __exit__(self, type, value, traceback):
        """Exit the fullsystem context manager.

        :param type: The type of exception that was raised
        :param value: The exception that was raised
        :param traceback: The traceback of the exception

        """
        if self.er_force_simulator_proc:
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

    CI_MODE_LAUNCH_DELAY_S = 0.3
    CI_MODE_PORT = 10009
    REFEREE_IP = "224.5.23.1"
    REFEREE_PORT = 10003
    CI_MODE_OUTPUT_RECEIVE_BUFFER_SIZE = 9000

    def __init__(self, supress_logs=False, ci_mode=False):
        """Run Gamecontroller

        :param supress_logs: Whether to suppress the logs
        :param ci_mode: Whether to run the gamecontroller in CI mode

        """
        self.supress_logs = supress_logs
        self.ci_mode = ci_mode

    def __enter__(self):
        """Enter the gamecontroller context manager. 

        :return: gamecontroller context managed instance

        """
        command = ["/opt/tbotspython/gamecontroller"]

        if self.ci_mode:
            command = ["/opt/tbotspython/gamecontroller", "--timeAcquisitionMode", "ci"]

        if self.supress_logs:
            with open(os.devnull, "w") as fp:
                self.gamecontroller_proc = Popen(command, stdout=fp, stderr=fp)

        else:
            self.gamecontroller_proc = Popen(command)

        if self.ci_mode:
            # We can't connect to the ci port right away, it takes
            # CI_MODE_LAUNCH_DELAY_S to start up the gamecontroller
            time.sleep(Gamecontroller.CI_MODE_LAUNCH_DELAY_S)

            self.ci_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.ci_socket.connect(("", Gamecontroller.CI_MODE_PORT))

        return self

    def __exit__(self, type, value, traceback):
        """Exit the gamecontroller context manager.

        :param type: The type of exception that was raised
        :param value: The exception that was raised
        :param traceback: The traceback of the exception

        """
        self.gamecontroller_proc.kill()
        self.gamecontroller_proc.wait()

        if self.ci_socket:
            self.ci_socket.close()

    def setup_proto_unix_io(
        self, blue_full_system_proto_unix_io, yellow_full_system_proto_unix_io
    ):
        """Setup gamecontroller io

        :param blue_full_system_proto_unix_io: The proto unix io of the blue full system.
        :param yellow_full_system_proto_unix_io: The proto unix io of the yellow full system.

        """

        def __send_referee_command(data):
            """Send a referee command from the gamecontroller to both full
            systems.

            :param data: The referee command to send

            """
            blue_full_system_proto_unix_io.send_proto(Referee, data)
            yellow_full_system_proto_unix_io.send_proto(Referee, data)

        self.receive_referee_command = SSLRefereeProtoListener(
            Gamecontroller.REFEREE_IP,
            Gamecontroller.REFEREE_PORT,
            __send_referee_command,
            True,
        )

    def send_ci_input(
        self,
        gc_command: proto.ssl_gc_state_pb2.Command,
        team: proto.ssl_gc_common_pb2.Team,
    ):
        """Send a ci input to the gamecontroller.

        CiInput -> Input -> Change ->  NewCommand -> Command -> (Type, Team)

        More info here:
        https://github.com/RoboCup-SSL/ssl-game-controller/blob/master/cmd/ssl-ci-test-client/README.md

        :param gc_command: The gc command to send
        :param team: The team to send the command to
        :return: The response CiOutput containing 1 or more refree msgs

        """
        ci_ci_input = CiInput(timestamp=int(time.time_ns()))
        ci_input = Input()
        ci_change = Change()
        ci_new_command = NewCommand()
        ci_command = Command(type=gc_command, for_team=team)

        ci_new_command.command.CopyFrom(ci_command)
        ci_change.new_command.CopyFrom(ci_new_command)
        ci_input.change.CopyFrom(ci_change)
        ci_ci_input.api_inputs.append(ci_input)

        # https://cwiki.apache.org/confluence/display/GEODE/Delimiting+Protobuf+Messages
        size = ci_ci_input.ByteSize()

        # Send a request to the host with the size of the message
        self.ci_socket.send(
            encoder._VarintBytes(size) + ci_ci_input.SerializeToString()
        )

        response_data = self.ci_socket.recv(
            Gamecontroller.CI_MODE_OUTPUT_RECEIVE_BUFFER_SIZE
        )

        msg_len, new_pos = decoder._DecodeVarint32(response_data, 0)
        ci_output = CiOutput()
        ci_output.ParseFromString(response_data[new_pos : new_pos + msg_len])
        return ci_output
