import os
import socket
import logging
import psutil
import time
import threading
import google.protobuf.internal.encoder as encoder
import google.protobuf.internal.decoder as decoder
import pdb

from subprocess import Popen
from software.python_bindings import *
from proto.import_all_protos import *
from software.py_constants import *
from extlibs.er_force_sim.src.protobuf.world_pb2 import (
    SimulatorState,
    SimBall,
    SimRobot,
)
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


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

    def __init__(
        self,
        full_system_runtime_dir=None,
        debug_full_system=False,
        friendly_colour_yellow=False,
        should_restart_on_crash=True,
    ):
        """Run FullSystem

        :param full_system_runtime_dir: The directory to run the blue full_system in
        :param debug_full_system: Whether to run the full_system in debug mode

        """
        self.full_system_runtime_dir = full_system_runtime_dir
        self.debug_full_system = debug_full_system
        self.friendly_colour_yellow = friendly_colour_yellow
        self.full_system_proc = None
        self.should_restart_on_crash = should_restart_on_crash

        self.thread = threading.Thread(target=self.__restart__)

    def __enter__(self):
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
        print("Full System running " + os.getcwd())

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

gdb --args bazel-bin/{full_system}

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

    def __restart__(self):
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

    def __exit__(self, type, value, traceback):
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

    def setup_proto_unix_io(self, proto_unix_io):
        """Helper to run full system and attach the appropriate unix senders/listeners

        :param proto_unix_io: The unix io to setup for this full_system instance

        """

        # Setup LOG(VISUALIZE) handling from full system. We set from_log_visualize
        # to true to decode from base64.
        for proto_class in [
            Obstacles,
            PathVisualization,
            PassVisualization,
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

    def __exit__(self, type, value, traceback):
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
            (SIMULATION_TICK_PATH, SimulatorTick),
            (WORLD_STATE_PATH, WorldState),
        ]:
            simulator_proto_unix_io.attach_unix_sender(self.simulator_runtime_dir, *arg)

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


class Gamecontroller(object):

    """ Gamecontroller Context Manager """

    CI_MODE_LAUNCH_DELAY_S = 0.3
    REFEREE_IP = "224.5.23.1"
    CI_MODE_OUTPUT_RECEIVE_BUFFER_SIZE = 9000

    def __init__(self, supress_logs=False, ci_mode=False):
        """Run Gamecontroller

        :param supress_logs: Whether to suppress the logs
        :param ci_mode: Whether to run the gamecontroller in CI mode

        """
        self.supress_logs = supress_logs
        self.ci_mode = ci_mode

        # We need to find 2 free ports to use for the gamecontroller
        # so that we can run multiple gamecontroller instances in parallel
        self.referee_port = self.next_free_port()
        self.ci_port = self.next_free_port()
        #self.ci_port = 10007

    def __enter__(self):
        """Enter the gamecontroller context manager. 

        :return: gamecontroller context managed instance

        """
        command = ["/opt/tbotspython/gamecontroller"]

        if self.ci_mode:
            command = ["/opt/tbotspython/gamecontroller", "--timeAcquisitionMode", "ci"]

        command += ["-publishAddress", f"{self.REFEREE_IP}:{self.referee_port}"]
        command += ["-ciAddress", f"localhost:{self.ci_port}"]

        if self.supress_logs:
            with open(os.devnull, "w") as fp:
                self.gamecontroller_proc = Popen(command, stdout=fp, stderr=fp)

        else:
            self.gamecontroller_proc = Popen(command)

        if self.ci_mode:
            print("ci port: " + str(self.ci_port))
            # We can't connect to the ci port right away, it takes
            # CI_MODE_LAUNCH_DELAY_S to start up the gamecontroller
            time.sleep(Gamecontroller.CI_MODE_LAUNCH_DELAY_S)

            self.ci_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.ci_socket.connect(("", self.ci_port))

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

    def next_free_port(self, port=40000, max_port=65535):
        """Find the next free port. We need to find 2 free ports to use for the gamecontroller
        so that we can run multiple gamecontroller instances in parallel.

        :param port: The port to start looking from
        :param max_port: The maximum port to look up to
        :return: The next free port

        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        while port <= max_port:
            try:
                sock.bind(("", port))
                sock.close()
                return port
            except OSError:
                port += 1

        raise IOError("no free ports")

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
            #print(data)
            #blue_full_system_proto_unix_io.send_proto(Referee, data)
            #yellow_full_system_proto_unix_io.send_proto(Referee, data)

        self.receive_referee_command = SSLRefereeProtoListener(
            Gamecontroller.REFEREE_IP, self.referee_port, __send_referee_command, True,
        )

    def send_ci_input(
        self,
        gc_command: proto.ssl_gc_state_pb2.Command,
        team: proto.ssl_gc_common_pb2.Team,
        final_ball_placement_point=None,
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

        # Do this only if ball placement pos is specified
        if final_ball_placement_point:
            # Set position
            ball_placement_pos = SetBallPlacementPos()
            ball_placement_pos.pos.CopyFrom(
                Vector2(
                    x=float(final_ball_placement_point.x()),
                    y=float(final_ball_placement_point.y()),
                )
            )
            ci_change = Change()
            ci_input = Input()
            ci_change.set_ball_placement_pos.CopyFrom(ball_placement_pos)
            ci_input.change.CopyFrom(ci_change)
            ci_ci_input.api_inputs.append(ci_input)

            # Start Placement
            ci_change = Change()
            ci_input = Input()
            start_placement = StartBallPlacement()
            ci_change.start_ball_placement.CopyFrom(start_placement)
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


class TigersAutoref(object):
    def __init__(self, ci_mode=False, autoref_runtime_dir=None, buffer_size=5, gc=Gamecontroller()):
        #pdb.set_trace()
        print("autoref init")
        self.tigers_autoref_proc = None
        self.auto_ref_proc_thread = None
        self.auto_ref_wrapper_thread = None
        self.ci_mode = ci_mode
        self.autoref_runtime_dir = autoref_runtime_dir
        self.wrapper_buffer = ThreadSafeBuffer(buffer_size, SSL_WrapperPacket)
        self.gamecontroller = gc
        self.ci_socket = self.gamecontroller.ci_socket
        #time.sleep(Gamecontroller.CI_MODE_LAUNCH_DELAY_S)
        #self.ci_port = self.gamecontroller.next_free_port()
        #print("autoref ci port: " + str(self.ci_port))
        #self.ci_socket.connect(("", self.ci_port))

    def __enter__(self):
        #pdb.set_trace()
        print("autoref enter")

        self.auto_ref_proc_thread = threading.Thread(target=self.startAutoref)
        self.auto_ref_proc_thread.start()

        self.auto_ref_wrapper_thread = threading.Thread(target=self.sslWrappers)
        self.auto_ref_wrapper_thread.start()
        print("thread started")

        return self

    def sslWrappers(self):
        print("ssl enter")
        #pdb.set_trace()
        while True:
            ssl_wrapper = self.wrapper_buffer.get(block=True)
            ci_input = AutoRefCiInput()
            ci_input.detection.append(ssl_wrapper.detection)
            #ci_input = AutoRefCiInput(detection=ssl_wrapper.detection)
            #ci_input.detection.CopyFrom(ssl_wrapper.detection)
            #self.autoref_sender.send_proto(ci_input)
            # https://cwiki.apache.org/confluence/display/GEODE/Delimiting+Protobuf+Messages
            size = ci_input.ByteSize()

            #self.autoref_sender.send_proto(ci_input)

            # Send a request to the host with the size of the message
            self.ci_socket.send(
                encoder._VarintBytes(size) + ci_input.SerializeToString()
            )
            response_data = self.ci_socket.recv(
                    Gamecontroller.CI_MODE_OUTPUT_RECEIVE_BUFFER_SIZE
            )

            #self.autoref_sender.send_proto(ci_input)

            #input()
            offset = 0
            while offset < len(response_data):
                msg_len, new_pos = decoder._DecodeVarint32(response_data, offset)
                offset = new_pos
                print("offset: " + str(offset) + ", " + str(msg_len) + ", " + str(new_pos) + ", " + str(len(response_data)))
                #print(response_data)
                ci_output = AutoRefCiOutput()
                if (offset + msg_len > len(response_data)):
                    print("Must get additional data")
                    response_data += self.ci_socket.recv(
                            offset + msg_len - len(response_data)
                    )
                ci_output.ParseFromString(response_data[offset : offset + msg_len])
                self.send_ci_input(ci_output.tracker_wrapper_packet)
                #print(ci_output)
                offset += msg_len

            #print(ci_input)

    def send_ci_input(self, tracker_wrapper):
        #pdb.set_trace()
        ci_input = CiInput(timestamp=int(time.time_ns()))
        ci_input.api_inputs.append(Input())
        ci_input.tracker_packet.CopyFrom(tracker_wrapper)

        # https://cwiki.apache.org/confluence/display/GEODE/Delimiting+Protobuf+Messages
        size = ci_input.ByteSize()

        # Send a request to the host with the size of the message
        self.gamecontroller.ci_socket.send(
            encoder._VarintBytes(size) + ci_input.SerializeToString()
        )

        print("response data from gamecontroller")
        response_data = self.gamecontroller.ci_socket.recv(
            Gamecontroller.CI_MODE_OUTPUT_RECEIVE_BUFFER_SIZE
        )
        #print(response_data)

        msg_len, new_pos = decoder._DecodeVarint32(response_data, 0)
        ci_output = CiOutput()
        ci_output.ParseFromString(response_data[new_pos : new_pos + msg_len])
        print("gamecontroller output")
        print(ci_output)


    def startAutoref(self):
        #pdb.set_trace()
        print(os.getcwd())
        os.chdir("/opt/tbotspython/autoReferee/")
        print(os.getcwd())
        autoref_cmd = "bin/./autoReferee -a -hl"

        if self.ci_mode:
            autoref_cmd += " -ci"

        #pdb.set_trace()
        #self.tigers_autoref_proc = Popen(autoref_cmd.split(" "))
        time.sleep(1)

    def setup_ssl_wrapper_packets(
        self, blue_fullsystem_proto_unix_io, yellow_fullsystem_proto_unix_io
    ):
        #pdb.set_trace()
        def __send_referee_command(data):
            """Send a referee command from the gamecontroller to both full
            systems.

            :param data: The referee command to send

            """
            print(data)
            blue_fullsystem_proto_unix_io.send_proto(Referee, data)
            yellow_fullsystem_proto_unix_io.send_proto(Referee, data)

        def __get_autoref_ci_output(data):
            print(data)
        
        #pdb.set_trace()
        blue_fullsystem_proto_unix_io.register_observer(SSL_WrapperPacket, self.wrapper_buffer)
        self.ci_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ci_socket.connect(("", 10013))

        #self.autoref_sender = SSL_AutoRefCiInputProtoSender("127.0.0.1", 10013, False)
        #self.autoref_receiver = SSL_AutoRefCiOutputProtoListener(
        #        "127.0.0.1", 10013, __get_autoref_ci_output, False,
        #)
        self.receive_referee_command = SSLRefereeProtoListener(
            Gamecontroller.REFEREE_IP, self.gamecontroller.ci_port, __send_referee_command, False,
        )


    def __exit__(self, type, value, traceback):
        if self.tigers_autoref_proc:
            self.tigers_autoref_proc.kill()
            self.tigers_autoref_proc.wait()

            self.auto_ref_proc_thread.join()
        self.auto_ref_wrapper_thread.join()
