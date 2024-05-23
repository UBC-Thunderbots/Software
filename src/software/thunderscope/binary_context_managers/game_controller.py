from __future__ import annotations

import logging
import os
import socket
import time
import google.protobuf.internal.encoder as encoder
import google.protobuf.internal.decoder as decoder
from subprocess import Popen
from typing import Any

from proto.import_all_protos import *
from proto.ssl_gc_common_pb2 import Team as SslTeam
from software.networking.ssl_proto_communication import *
import software.python_bindings as tbots_cpp
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.python_bindings import *
from software.py_constants import *
from software.thunderscope.binary_context_managers.util import *
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class Gamecontroller(object):
    """ Gamecontroller Context Manager """

    CI_MODE_LAUNCH_DELAY_S = 0.3
    REFEREE_IP = "224.5.23.1"
    CI_MODE_OUTPUT_RECEIVE_BUFFER_SIZE = 9000

    def __init__(self, supress_logs: bool = False, gamecontroller_port:int =None, referee_addresse:int= None) -> None:
        """Run Gamecontroller

        :param supress_logs: Whether to suppress the logs
        :param gamecontroller_port: the port that the gamecontroller would bind to 
        :param referee_address: the address the referee binds to
        """
        self.supress_logs = supress_logs

        # We need to find 2 free ports to use for the gamecontroller
        # so that we can run multiple gamecontroller instances in parallel
        self.referee_port = self.next_free_port()

        self.REFEREE_IP = referee_addresse
        if referee_addresse is None: 
            self.REFEREE_IP = "224.5.23.1"

        self.ci_port = gamecontroller_port
        if self.ci_port is None:
            self.ci_port = self.next_free_port(40000)
        else:
            self.ci_port = self.next_free_port(self.ci_port)

        # this allows gamecontroller to listen to override commands
        self.command_override_buffer = ThreadSafeBuffer(
            buffer_size=2, protobuf_type=ManualGCCommand
        )

    def __enter__(self) -> "self":
        """Enter the gamecontroller context manager. 

        :return: gamecontroller context managed instance

        """
        command = ["/opt/tbotspython/gamecontroller", "--timeAcquisitionMode", "ci"]

        command += ["-publishAddress", f"{self.REFEREE_IP}:{self.referee_port}"]
        command += ["-ciAddress", f"localhost:{self.ci_port}"]

        if self.supress_logs:
            with open(os.devnull, "w") as fp:
                self.gamecontroller_proc = Popen(command, stdout=fp, stderr=fp)

        else:
            self.gamecontroller_proc = Popen(command)

        # We can't connect to the ci port right away, it takes
        # CI_MODE_LAUNCH_DELAY_S to start up the gamecontroller
        time.sleep(Gamecontroller.CI_MODE_LAUNCH_DELAY_S)
        self.ci_socket = SslSocket(self.ci_port)

        return self

    def __exit__(self, type, value, traceback) -> None:
        """Exit the gamecontroller context manager.

        :param type: The type of exception that was raised
        :param value: The exception that was raised
        :param traceback: The traceback of the exception

        """
        self.gamecontroller_proc.terminate()
        self.gamecontroller_proc.wait()

        self.ci_socket.close()

    def refresh(self):
        """
        Gets any manual gamecontroller commands from the buffer and executes them
        """
        manual_command = self.command_override_buffer.get(return_cached=False)

        while manual_command is not None:
            self.send_gc_command(
                gc_command=manual_command.manual_command.type,
                team=manual_command.manual_command.for_team,
                final_ball_placement_point=tbots_cpp.Point(
                    manual_command.final_ball_placement_point.x,
                    manual_command.final_ball_placement_point.y,
                )
                # HasField checks if the field was manually set by us
                # as opposed to if a value exists (since a default value always exists)
                if manual_command.HasField("final_ball_placement_point") else None,
            )
            manual_command = self.command_override_buffer.get(return_cached=False)

    def is_valid_port(self, port:int) -> bool:
        """
        Check if a port is available 

        :param port: the port we are checking
        :return: True if the port is available, False otherwise
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        try:
            sock.bind(("", port))
            sock.close()
            return True
        except OSError:
            return False

    def next_free_port(self, start_port: int = 40000, max_port: int = 65535) -> int:
        """Find the next free port. We need to find 2 free ports to use for the gamecontroller
        so that we can run multiple gamecontroller instances in parallel.

        :param start_port: The port to start looking from
        :param max_port: The maximum port to look up to
        :return: The next free port

        """
        while start_port <= max_port:
            if self.is_valid_port(start_port):
                return start_port
            start_port += 1

        raise IOError("no free ports")

    def setup_proto_unix_io(
        self,
        blue_full_system_proto_unix_io: ProtoUnixIO,
        yellow_full_system_proto_unix_io: ProtoUnixIO,
        autoref_proto_unix_io: ProtoUnixIO = None,
    ) -> None:
        """Setup gamecontroller io

        :param blue_full_system_proto_unix_io: The proto unix io of the blue full system.
        :param yellow_full_system_proto_unix_io: The proto unix io of the yellow full system.
        :param autoref_proto_unix_io: The proto unix io for the autoref

        """

        def __send_referee_command(data: Referee) -> None:
            """Send a referee command from the gamecontroller to both full
            systems.

            :param data: The referee command to send

            """
            blue_full_system_proto_unix_io.send_proto(Referee, data)
            yellow_full_system_proto_unix_io.send_proto(Referee, data)
            if autoref_proto_unix_io is not None:
                autoref_proto_unix_io.send_proto(Referee, data)

        self.receive_referee_command = tbots_cpp.SSLRefereeProtoListener(
            self.REFEREE_IP, self.referee_port, __send_referee_command, True,
        )

        blue_full_system_proto_unix_io.register_observer(
            ManualGCCommand, self.command_override_buffer
        )
        yellow_full_system_proto_unix_io.register_observer(
            ManualGCCommand, self.command_override_buffer
        )

    def send_gc_command(
        self,
        gc_command: proto.ssl_gc_state_pb2.Command,
        team: proto.ssl_gc_common_pb2.Team,
        final_ball_placement_point: tbots_cpp.Point = None,
    ) -> Any:
        """Send a ci input to the gamecontroller.

        CiInput -> Input -> Change ->  NewCommand -> Command -> (Type, Team)

        More info here:
        https://github.com/RoboCup-SSL/ssl-game-controller/blob/master/cmd/ssl-ci-test-client/README.md

        :param gc_command: The gc command to send
        :param team: The team to send the command to
        :param final_ball_placement_point: ball placement point for BallPlacement messages
        :return: The response CiOutput containing 1 or more refree msgs

        """
        ci_input = CiInput(timestamp=int(time.time_ns()))
        api_input = Input()
        change = Change()
        new_command = NewCommand()
        command = Command(type=gc_command, for_team=team)

        new_command.command.CopyFrom(command)
        change.new_command.CopyFrom(new_command)
        api_input.change.CopyFrom(change)
        ci_input.api_inputs.append(api_input)

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
            change = Change()
            api_input = Input()
            change.set_ball_placement_pos.CopyFrom(ball_placement_pos)
            api_input.change.CopyFrom(change)
            ci_input.api_inputs.append(api_input)

            # Start Placement
            change = Change()
            api_input = Input()
            start_placement = StartBallPlacement()
            change.start_ball_placement.CopyFrom(start_placement)
            api_input.change.CopyFrom(change)
            ci_input.api_inputs.append(api_input)

        ci_output_list = self.send_ci_input(ci_input)

        return ci_output_list

    def send_ci_input(self, ci_input: proto.ssl_gc_ci_pb2.CiInput) -> list[CiOutput]:
        """
        Send CiInput proto to the Gamecontroller. Retries if the Gamecontroller output isn't parseable as a CiOutput proto

        :param CiInput proto to send to the Gamecontroller

        :return: a list of CiOutput protos received from the Gamecontroller
        """
        ci_output_list = list()
        while True:
            try:
                self.ci_socket.send(ci_input)
                ci_output_list = self.ci_socket.receive(CiOutput)
                break
            except SslSocketProtoParseException as parse_err:
                logging.info(
                    "error receiving CiOutput proto from the gamecontroller: "
                    + parse_err.args
                )

        return ci_output_list

    def reset_team(self, name: str, team: str) -> UpdateTeamState:
        """
        Returns an UpdateTeamState proto for the gamecontroller to reset team info.

        :param name name of the new team
        :param team yellow or blue team to update

        :return: corresponding UpdateTeamState proto
        """
        update_team_state = UpdateTeamState()
        update_team_state.for_team = team
        update_team_state.team_name = name
        update_team_state.goals = 0
        update_team_state.timeouts_left = 4
        update_team_state.timeout_time_left = "05:00"
        update_team_state.can_place_ball = True

        return update_team_state

    def reset_game(self, division: proto.ssl_gc_common_pb2.Division) -> UpdateConfig:
        """
        Returns an UpdateConfig proto for the Gamecontroller to reset game info.

        :param division the Division proto corresponding to the game division to set up the Gamecontroller for

        :return: corresponding UpdateConfig proto
        """
        game_update = UpdateConfig()
        game_update.division = division
        game_update.first_kickoff_team = SslTeam.BLUE
        game_update.auto_continue = True
        game_update.match_type = MatchType.FRIENDLY

        return game_update

    def reset_team_info(
        self, division: proto.ssl_gc_common_pb2.Division
    ) -> list[CiOutput]:
        """
        Sends a message to the Gamecontroller to reset Team information.

        :param division: the Division proto corresponding to the game division to set up the Gamecontroller for

        :return: a list of CiOutput protos from the Gamecontroller
        """
        ci_input = CiInput(timestamp=int(time.time_ns()))
        input_blue_update = Input()
        input_blue_update.reset_match = True
        input_blue_update.change.update_team_state.CopyFrom(
            self.reset_team("BLUE", SslTeam.BLUE)
        )

        input_yellow_update = Input()
        input_yellow_update.reset_match = True
        input_yellow_update.change.update_team_state.CopyFrom(
            self.reset_team("YELLOW", SslTeam.YELLOW)
        )

        input_game_update = Input()
        input_game_update.reset_match = True
        input_game_update.change.update_config.CopyFrom(self.reset_game(division))

        ci_input.api_inputs.append(input_blue_update)
        ci_input.api_inputs.append(input_yellow_update)
        ci_input.api_inputs.append(input_game_update)

        return self.send_ci_input(ci_input)

    def update_game_engine_config(
        self, config: proto.ssl_gc_engine_config_pb2
    ) -> list[CiOutput]:
        """
        Sends a game engine config update.

        :param config: the new SSL game engine config

        :return: a list of CiOutput protos from the Gamecontroller
        """
        ci_input = CiInput(timestamp=int(time.time_ns()))

        game_config_input = Input()
        game_config_input.config_delta.CopyFrom(config)

        ci_input.api_inputs.append(game_config_input)

        return self.send_ci_input(ci_input)
