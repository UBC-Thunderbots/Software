import os
import socket
import logging
import psutil
import time
import threading
import google.protobuf.internal.encoder as encoder
import google.protobuf.internal.decoder as decoder

from subprocess import Popen
from software.python_bindings import *
import software.python_bindings as tbots
from proto.import_all_protos import *
from proto.ssl_gc_common_pb2 import Team
from software.py_constants import *
from extlibs.er_force_sim.src.protobuf.world_pb2 import (
    SimulatorState,
)
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.networking.ssl_proto_communication import *

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

    def __init__(
        self, simulator_runtime_dir=None, debug_simulator=False, enable_realism=False
    ):
        """Run Simulator

        NOTE: If any of the runtime directories are None, the corresponding binary
              will not be launched.

        :param simulator_runtime_dir: The directory to run the simulator in
        :param debug_simulator: Whether to run the simulator in debug mode

        """
        self.simulator_runtime_dir = simulator_runtime_dir
        self.debug_simulator = debug_simulator
        self.er_force_simulator_proc = None
        self.enable_realism = enable_realism

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

    def setup_autoref_proto_unix_io(self, autoref_proto_unix_io):
        """
        Setup the proto unix io for the autoref, so that it can receive tracker wrapper packets

        :param autoref_proto_unix_io the proto unix io for the autoref

        """
        autoref_proto_unix_io.attach_unix_receiver(
                self.simulator_runtime_dir, SSL_WRAPPER_PATH, SSL_WrapperPacket
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
            # We can't connect to the ci port right away, it takes
            # CI_MODE_LAUNCH_DELAY_S to start up the gamecontroller
            time.sleep(Gamecontroller.CI_MODE_LAUNCH_DELAY_S)

            self.ci_socket = SslSocket(self.ci_port)

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

        register_referee_command_observer(blue_full_system_proto_unix_io,
                                          yellow_full_system_proto_unix_io,
                                          self.referee_port)

    def register_referee_command_observer(
            self, blue_full_system_proto_unix_io, yellow_full_system_proto_unix_io,
            port
            ):
        def __send_referee_command(data):
            """Send a referee command from the gamecontroller to both full
            systems.

            :param data: The referee command to send

            """
            blue_full_system_proto_unix_io.send_proto(Referee, data)
            yellow_full_system_proto_unix_io.send_proto(Referee, data)

        self.receive_referee_command = SSLRefereeProtoListener(
            Gamecontroller.REFEREE_IP, port, __send_referee_command, True,
        )

    def register_ci_referee_command_observer(
            self, blue_full_system_proto_unix_io, yellow_full_system_proto_unix_io
            ):
        self.register_referee_command_observer(blue_full_system_proto_unix_io,
                                               yellow_full_system_proto_unix_io,
                                               self.ci_port)

    def send_gc_command(
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

        ci_output_list = self.send_ci_input(ci_ci_input)

        return ci_output_list

    def send_ci_input(self, ci_input):
        '''
        Send CiInput proto to the Gamecontroller. Retries if the Gamecontroller output isn't parseable as a CiOutput proto

        :param CiInput proto to send to the Gamecontroller

        :return: a list of CiOutput protos received from the Gamecontroller
        '''
        ci_output_list = list()
        while True :
            try :
                self.ci_socket.send(ci_input)
                ci_output_list = self.ci_socket.receive(CiOutput)
                break
            except SslSocketProtoParseException as parse_err:
                logging.info("error receiving CiOutput proto from the gamecontroller: " + parse_err.args)
                pass

        return ci_output_list

    def reset_team(self, name, team):
        '''
        Returns an UpdateTeamState proto for the gamecontroller to reset team info.

        :param name name of the new team
        :param team yellow or blue team to update

        :return: corresponding UpdateTeamState proto
        '''
        update_team_state                   = UpdateTeamState()
        update_team_state.for_team          = team
        update_team_state.team_name         = name
        update_team_state.goals             = 0
        update_team_state.timeouts_left     = 4
        update_team_state.timeout_time_left = "05:00" 
        update_team_state.can_place_ball    = True

        return update_team_state

    def reset_game(self, division):
        '''
        Returns an UpdateConfig proto for the Gamecontroller to reset game info.

        :param division the Division proto corresponding to the game division to set up the Gamecontroller for

        :return: corresponding UpdateConfig proto
        '''
        game_update = UpdateConfig() 
        game_update.division = division 
        game_update.first_kickoff_team = Team.BLUE
        game_update.auto_continue = True
        game_update.match_type = MatchType.FRIENDLY

        return game_update

    def reset_team_info(self, division):
        '''
        Sends a message to the Gamecontroller to reset Team information.

        :param division the Division proto corresponding to the game division to set up the Gamecontroller for

        :return: a list of CiOutput protos from the Gamecontroller
        '''
        ci_ci_input = CiInput(timestamp=int(time.time_ns()))
        ci_input_blue_update = Input()
        ci_input_blue_update.reset_match = True
        ci_input_blue_update.change.update_team_state.CopyFrom(self.reset_team("BLUE", Team.BLUE))

        ci_input_yellow_update = Input()
        ci_input_yellow_update.reset_match = True
        ci_input_yellow_update.change.update_team_state.CopyFrom(self.reset_team("YELLOW", Team.YELLOW))

        ci_input_game_update = Input()
        ci_input_game_update.reset_match = True
        ci_input_game_update.change.update_config.CopyFrom(self.reset_game(division))

        ci_ci_input.api_inputs.append(ci_input_blue_update)
        ci_ci_input.api_inputs.append(ci_input_yellow_update)
        ci_ci_input.api_inputs.append(ci_input_game_update)

        return self.send_ci_input(ci_ci_input)

class TigersAutoref(object):
    '''
    A wrapper over the TigersAutoref binary. It coordinates communication between the Simulator, TigersAutoref and Gamecontroller. 

    In CI mode, the flow of data corresponds to:

              SSL_DetectionFrame          AutoRefCiOutput: TrackerWrapperPacket                CiOutput
    Simulator ------------------> AutoRef ------------------------------------> Gamecontroller --------------> AI
                AutoRefCiInput                         CiInput                                 RefereeMessage

    For more information: https://github.com/RoboCup-SSL/ssl-game-controller/blob/master/doc/AutoRefCi.md
    '''
    AUTOREF_COMM_PORT = 10013
    AUTOREF_NUM_RETRIES = 10

    def __init__(self, ci_mode=False, autoref_runtime_dir=None, buffer_size=5, gc=Gamecontroller(), supress_logs=True):
        self.tigers_autoref_proc = None
        self.auto_ref_proc_thread = None
        self.auto_ref_wrapper_thread = None
        self.ci_mode = ci_mode
        self.autoref_runtime_dir = autoref_runtime_dir
        self.wrapper_buffer = ThreadSafeBuffer(buffer_size, SSL_WrapperPacket)
        self.gamecontroller = gc
        self.supress_logs = supress_logs

    def __enter__(self):
        if not os.path.exists("/opt/tbotspython/autoReferee/bin/autoReferee"):
            logging.info("Could not find autoref binary, did you run ./setup_software.sh")

        self.auto_ref_proc_thread = threading.Thread(target=self.start_autoref, daemon=True)
        self.auto_ref_proc_thread.start()

        self.auto_ref_wrapper_thread = threading.Thread(target=self.send_to_autoref_and_forward_to_gamecontroller)
        self.auto_ref_wrapper_thread.start()

        return self

    def send_geometry(self):
        '''
        Sends updated field geometry to the AutoRef so that the TigersAutoref knows about field sizes.
        '''
        ssl_wrapper = self.wrapper_buffer.get(block=True)
        ci_input = AutoRefCiInput()
        ci_input.detection.append(ssl_wrapper.detection)

        field = tbots.Field.createSSLDivisionBField()  
        ci_input.geometry.CopyFrom(createGeometryData(field, 0.3))

        self.ci_socket.send(ci_input)

        response_data = None
        while True:
            try :
                response_data = self.ci_socket.receive(AutoRefCiOutput)
                break
            except SslSocketProtoParseException as parse_err:
                logging.info("error with sending geometry data: \n" + parse_err.args)
                pass

        for ci_output in response_data:
            self.forward_to_gamecontroller(ci_output.tracker_wrapper_packet)

    def persistently_connect_to_autoref(self):
        '''
        Connect to the TigersAutoref binary. Retry connection a few times if the connection doesn't go through in case the binary hasn't started yet.

        :return: True if the action was successful, False otherwise
        '''
        tries = 0
        while (tries < TigersAutoref.AUTOREF_NUM_RETRIES):
            try :
                # We cannot start the Autoref binary immediately, so we must wait until the binary has started before we try to connect to it
                time.sleep(0.5);
                self.ci_socket = SslSocket(TigersAutoref.AUTOREF_COMM_PORT)
                return True
            except ConnectionRefusedError:
                tries += 1

        return False

    def send_to_autoref_and_forward_to_gamecontroller(self):
        '''
        Main communication loop that sets up the TigersAutoref and coordinates communication between Simulator, TigersAutoref and Gamecontroller. Returns early if connection to the TigersAutoref binary was unsuccessful.
        '''
        if not self.persistently_connect_to_autoref():
            logging.info("Failed to connect to autoref binary. Is it running?")
            return


        self.send_geometry();
        self.gamecontroller.resetTeamInfo(Division.DIV_B)

        self.gamecontroller.send_gc_command(
            gc_command=Command.Type.STOP, team=Team.UNKNOWN
        )

        while True:
            try :
                ssl_wrapper = self.wrapper_buffer.get(block=True)
                ci_input = AutoRefCiInput()
                ci_input.detection.append(ssl_wrapper.detection)

                self.ci_socket.send(ci_input)
                response_data = self.ci_socket.receive(AutoRefCiOutput)

                for ci_output in response_data:
                    self.forward_to_gamecontroller(ci_output.tracker_wrapper_packet)
            except SslSocketProtoParseException as parse_error :
                logging.info("error with receiving AutoRefCiOutput, ignoring this packet...")
                pass


    def forward_to_gamecontroller(self, tracker_wrapper):
        '''
        Uses the given tracker_wrapper to create a CiInput for the Gamecontroller to track. Uses the timestmap from the given tracker_wrapper to support asynchronous ticking.

        :param tracker_wrapper TrackerWrapperPacket for the Gamecontroller to track

        :return: a list of CiOutput protos received from the Gamecontroller
        '''
        ci_input = CiInput(timestamp=int(tracker_wrapper.tracked_frame.timestamp*NANOSECONDS_PER_SECOND))
        ci_input.api_inputs.append(Input())
        ci_input.tracker_packet.CopyFrom(tracker_wrapper)

        return self.gamecontroller.send_ci_input(ci_input)

    def start_autoref(self):
        '''
        Starts the TigersAutoref binary.
        '''
        autoref_cmd = "software/autoref/run_autoref"

        if self.ci_mode:
            autoref_cmd += " --ci"

        if self.supress_logs:
            with open(os.devnull, "w") as fp:
                self.tigers_autoref_proc = Popen(autoref_cmd.split(' '), stdout=fp, stderr=fp)
        else:
            self.tigers_autoref_proc = Popen(autoref_cmd.split(' '))

    def setup_ssl_wrapper_packets(
        self, autoref_proto_unix_io,
        blue_full_system_proto_unix_io,
        yellow_full_system_proto_unix_io
    ):
        '''
        Registers as an observer of TrackerWrapperPackets from the Simulator, so that they can be forwarded to the Gamecontroller in CI mode.
        '''
        autoref_proto_unix_io.register_observer(SSL_WrapperPacket, self.wrapper_buffer)
        self.gamecontroller.register_ci_referee_command_observer(blue_full_system_proto_unix_io,
                                                                 yellow_full_system_proto_unix_io)


    def __exit__(self, type, value, traceback):
        if self.tigers_autoref_proc:
            self.tigers_autoref_proc.kill()
            self.tigers_autoref_proc.wait()

            self.auto_ref_proc_thread.join()
        self.auto_ref_wrapper_thread.join()
