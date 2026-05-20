from __future__ import annotations
import itertools

import queue
import random
import logging
import os
import time
from subprocess import Popen
from typing import Any

from proto.import_all_protos import *
from proto.message_translation.tbots_protobuf import create_default_world_state
from proto.ssl_gc_common_pb2 import Team as SslTeam
from software.networking.ssl_proto_communication import *
import software.python_bindings as tbots_cpp
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.python_bindings import *
from software.py_constants import *
from software.thunderscope.binary_context_managers.util import *
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.common.thread_safe_circular_buffer import (
    ThreadSafeCircularBuffer,
)
from software.thunderscope.util import is_current_platform_macos
from software.thunderscope.time_provider import time_provider_instance


class Gamecontroller:
    """Gamecontroller Context Manager"""

    CI_MODE_LAUNCH_DELAY_S = 0.1
    CI_MODE_CONNECT_TIMEOUT_S = 5.0
    CI_MODE_OUTPUT_RECEIVE_BUFFER_SIZE = 9000
    REFEREE_IP = "224.5.23.1"
    RESET_MATCH_DELAY_S = 1
    NO_GAME_PROGRESS_DURATION_S = 120

    def __init__(
        self,
        suppress_logs: bool = False,
        use_conventional_port: bool = False,
        automate_referee: bool = False,
    ) -> None:
        """Run Gamecontroller

        :param suppress_logs: Whether to suppress the logs
        :param use_conventional_port: whether or not to use the conventional port!
        :param automate_referee: whether or not referee commands should be automated
        """
        self.suppress_logs = suppress_logs
        self.automate_referee = automate_referee

        # We default to using a non-conventional port to avoid emitting
        # on the same port as what other teams may be listening on.
        if use_conventional_port:
            if not self.is_valid_port(SSL_REFEREE_PORT):
                raise OSError(f"Cannot use port {SSL_REFEREE_PORT} for Gamecontroller")

            self.referee_port = SSL_REFEREE_PORT
        else:
            self.referee_port = self.next_free_port(random.randint(1024, 65535))

        self.ci_port = self.next_free_port()
        # this allows gamecontroller to listen to override commands
        self.command_override_buffer = ThreadSafeBuffer(
            buffer_size=2, protobuf_type=ManualGCCommand
        )

        self.simulator_proto_unix_io = None
        self.blue_team_world_buffer = ThreadSafeCircularBuffer(
            buffer_size=1, protobuf_type=World
        )
        self.latest_world = None
        self.blue_removed_robot_ids = queue.Queue()
        self.yellow_removed_robot_ids = queue.Queue()
        self.processed_event_ids = set()
        self.last_stage_time_left = None
        self.pause_start_timestamp = None

    def get_referee_port(self) -> int:
        """Sometimes, the port that we are using changes depending on context.
        We want a getter function that returns the port we are using.

        :return: the port that the game controller is currently using!
        """
        return self.referee_port

    def __enter__(self) -> Gamecontroller:
        """Enter the gamecontroller context manager.

        :return: gamecontroller context managed instance
        """
        command = ["/opt/tbotspython/gamecontroller", "--timeAcquisitionMode", "ci"]

        # Kill any gamecontroller, even those with different IP/port, because
        # of the port assignment logic when use_conventional_port=False
        kill_cmd_if_running(command)

        command += ["-publishAddress", f"{self.REFEREE_IP}:{self.referee_port}"]
        command += ["-ciAddress", f"localhost:{self.ci_port}"]

        # Run in a temp dir so the gamecontroller always starts with a clean
        # state store — the state-store.json.stream can get corrupted when the
        # process is killed mid-write, causing a panic on the next startup.
        import tempfile
        self._gc_workdir = tempfile.mkdtemp(prefix="gamecontroller_")

        if self.suppress_logs:
            self._gc_log = open(
                os.path.join(self._gc_workdir, "gamecontroller.log"), "w"
            )
            self.gamecontroller_proc = Popen(
                command, stdout=self._gc_log, stderr=self._gc_log,
                cwd=self._gc_workdir,
            )
        else:
            self._gc_log = None
            self.gamecontroller_proc = Popen(command, cwd=self._gc_workdir)

        # Poll until the gamecontroller CI port is ready, up to CI_MODE_CONNECT_TIMEOUT_S
        deadline = time.time() + Gamecontroller.CI_MODE_CONNECT_TIMEOUT_S
        while True:
            if self.gamecontroller_proc.poll() is not None:
                log_contents = ""
                log_path = os.path.join(self._gc_workdir, "gamecontroller.log")
                if os.path.exists(log_path):
                    with open(log_path) as f:
                        log_contents = f.read()
                raise RuntimeError(
                    f"Gamecontroller process exited with code {self.gamecontroller_proc.returncode} "
                    f"before CI port {self.ci_port} became available"
                    + (f"\nGamecontroller output:\n{log_contents}" if log_contents else "")
                )
            try:
                self.ci_socket = SslSocket(self.ci_port)
                break
            except ConnectionRefusedError:
                if time.time() >= deadline:
                    raise
                time.sleep(Gamecontroller.CI_MODE_LAUNCH_DELAY_S)

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
        if self._gc_log is not None:
            self._gc_log.close()
        import shutil
        shutil.rmtree(self._gc_workdir, ignore_errors=True)

    def refresh(self):
        """Gets any manual gamecontroller commands from the buffer and executes them"""
        manual_command = self.command_override_buffer.get(return_cached=False)

        while manual_command is not None:
            self.send_gc_command(
                gc_command=manual_command.manual_command.type,
                team=manual_command.manual_command.for_team,
                final_ball_placement_point=(
                    tbots_cpp.Point(
                        manual_command.final_ball_placement_point.x,
                        manual_command.final_ball_placement_point.y,
                    )
                    # HasField checks if the field was manually set by us
                    # as opposed to if a value exists (since a default value always exists)
                    if manual_command.HasField("final_ball_placement_point")
                    else None
                ),
            )
            manual_command = self.command_override_buffer.get(return_cached=False)

    def is_valid_port(self, port):
        """Determine whether or not a given port is valid

        :param port: the port we are checking
        :return: True if a port is valid False otherwise
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
        simulator_proto_unix_io: ProtoUnixIO = None,
    ) -> None:
        """Setup gamecontroller io

        :param blue_full_system_proto_unix_io: The proto unix io of the blue full system.
        :param yellow_full_system_proto_unix_io: The proto unix io of the yellow full system.
        :param autoref_proto_unix_io: The proto unix io for the autoref
        :param simulator_proto_unix_io: The socket for the Gamecontroller to interact with the simulator
        """

        def __send_referee_command(data: Referee) -> None:
            """Send a referee command from the gamecontroller to both full
            systems.

            :param data: The referee command to send
            """
            self.__handle_referee(data)
            blue_full_system_proto_unix_io.send_proto(Referee, data)
            yellow_full_system_proto_unix_io.send_proto(Referee, data)
            if autoref_proto_unix_io is not None:
                autoref_proto_unix_io.send_proto(Referee, data)

        if is_current_platform_macos():
            loopback_iface = "en0"
        else:
            loopback_iface = "lo"

        self.receive_referee_command = tbots_cpp.SSLRefereeProtoListener(
            Gamecontroller.REFEREE_IP,
            self.referee_port,
            loopback_iface,
            __send_referee_command,
            True,
        )

        blue_full_system_proto_unix_io.register_observer(
            ManualGCCommand, self.command_override_buffer
        )
        yellow_full_system_proto_unix_io.register_observer(
            ManualGCCommand, self.command_override_buffer
        )
        blue_full_system_proto_unix_io.register_observer(
            World, self.blue_team_world_buffer
        )
        self.simulator_proto_unix_io = simulator_proto_unix_io

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
        new_command = Change.NewCommand()
        command = Command(type=gc_command, for_team=team)

        new_command.command.CopyFrom(command)
        change.new_command_change.CopyFrom(new_command)
        api_input.change.CopyFrom(change)
        ci_input.api_inputs.append(api_input)

        # Do this only if ball placement pos is specified
        if final_ball_placement_point:
            # Set position
            ball_placement_pos = Change.SetBallPlacementPos()
            ball_placement_pos.pos.CopyFrom(
                Vector2(
                    x=float(final_ball_placement_point.x()),
                    y=float(final_ball_placement_point.y()),
                )
            )
            change = Change()
            api_input = Input()
            change.set_ball_placement_pos_change.CopyFrom(ball_placement_pos)
            api_input.change.CopyFrom(change)
            ci_input.api_inputs.append(api_input)

            # Start Placement
            api_input = Input()
            start_placement = ContinueAction()
            start_placement.type = ContinueAction.Type.BALL_PLACEMENT_START
            start_placement.for_team = team
            api_input.continue_action.CopyFrom(start_placement)
            ci_input.api_inputs.append(api_input)

        ci_output_list = self.send_ci_input(ci_input)

        return ci_output_list

    def send_ci_input(self, ci_input: proto.ssl_gc_ci_pb2.CiInput) -> list[CiOutput]:
        """Send CiInput proto to the Gamecontroller. Retries if the Gamecontroller output isn't parseable as a CiOutput proto

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
            except OSError:
                break

        return ci_output_list

    def reset_match(self) -> list[CiOutput]:
        """Sends a message to the Gamecontroller to reset match information to our defaults.

        :return: a list of CiOutput protos from the Gamecontroller
        """
        ci_input = CiInput(timestamp=int(time.time_ns()))

        input_reset_match = Input()
        input_reset_match.reset_match = True

        input_set_match_config = Input()
        input_set_match_config.change.update_config_change.division = Division.DIV_B
        input_set_match_config.change.update_config_change.match_type = (
            MatchType.FRIENDLY
        )

        ci_input.api_inputs.append(input_reset_match)
        ci_input.api_inputs.append(input_set_match_config)

        return self.send_ci_input(ci_input)

    def update_game_engine_config(
        self, config: proto.ssl_gc_engine_config_pb2
    ) -> list[CiOutput]:
        """Sends a game engine config update.

        :param config: the new SSL game engine config

        :return: a list of CiOutput protos from the Gamecontroller
        """
        ci_input = CiInput(timestamp=int(time.time_ns()))

        game_config_input = Input()
        game_config_input.config_delta.CopyFrom(config)

        ci_input.api_inputs.append(game_config_input)

        return self.send_ci_input(ci_input)

    def __handle_referee(self, referee: Referee) -> None:
        """Updates the world state based on the referee message
        :param referee: the referee protobuf message
        """
        # Check that we are running with the simulator and have access to its
        # proto unix io
        if self.simulator_proto_unix_io is None:
            return

        # Convert the latest blue world into a WorldState we can send to the simulator and update the robots
        self.latest_world = self.blue_team_world_buffer.get(
            block=False, return_cached=True
        )

        if self.automate_referee:
            self.__automate_referee(referee)

        if (
            len(self.latest_world.friendly_team.team_robots)
            != referee.blue.max_allowed_bots
            or len(self.latest_world.enemy_team.team_robots)
            != referee.yellow.max_allowed_bots
        ):
            self.__update_max_allowed_robots(referee)

    def __automate_referee(self, referee: Referee) -> None:
        """Automate referee events by handling possible goals, ball placement failures,
        game stage changes, starting new game, and resetting when there is no game progress.

        :param referee: the referee protobuf message
        """
        self.__handle_game_no_progress(referee)

        if referee.stage_time_left < 0:
            self.__handle_game_stage_change(referee.stage)

        possible_goal_events = self.__find_unprocessed_events(
            referee, GameEvent.Type.POSSIBLE_GOAL
        )
        placement_failed_events = self.__find_unprocessed_events(
            referee, GameEvent.Type.PLACEMENT_FAILED
        )

        if len(possible_goal_events) >= 1:
            self.__handle_possible_goal(possible_goal_events[0].possible_goal.by_team)
            self.processed_event_ids.add(possible_goal_events[0].id)

        if len(placement_failed_events) >= 2:
            self.__handle_placement_failed()
            self.processed_event_ids.add(placement_failed_events[0].id)
            self.processed_event_ids.add(placement_failed_events[1].id)

    def __handle_game_no_progress(self, referee: Referee) -> None:
        """Handle game stalling by ending early if there is no progress in game stage time for too long.

        :param referee: the referee protobuf message
        """
        is_game_progressing = referee.stage_time_left != self.last_stage_time_left
        self.last_stage_time_left = referee.stage_time_left

        if is_game_progressing:
            self.pause_start_timestamp = None
            return

        # First referee message after being paused
        if self.pause_start_timestamp is None:
            self.pause_start_timestamp = time_provider_instance.elapsed_time_ns()

        pause_duration_s = (
            time_provider_instance.elapsed_time_ns() - self.pause_start_timestamp
        ) * SECONDS_PER_NANOSECOND

        if pause_duration_s >= self.NO_GAME_PROGRESS_DURATION_S:
            logging.warning(
                f"Gamecontroller: Ending game early due unknown issue halting "
                f"game progress for {self.NO_GAME_PROGRESS_DURATION_S} seconds"
            )
            self.__handle_game_stage_change(Referee.Stage.NORMAL_SECOND_HALF)
            self.pause_start_timestamp = None

    def __handle_game_stage_change(self, game_stage: Referee.Stage) -> None:
        """Handle game stage change by advancing to the next half once first half is over
        and starting a new game once the second half is over.

        :param game_stage: The current game stage from the referee message
        """
        if game_stage == Referee.Stage.NORMAL_FIRST_HALF:
            # skip to pre second half
            self.__set_game_stage(Referee.Stage.NORMAL_SECOND_HALF_PRE)
        else:
            # end game
            self.__set_game_stage(Referee.Stage.POST_GAME)

            # Race condition where events don't get resolved before setting new match settings
            time.sleep(self.RESET_MATCH_DELAY_S)

            # start new game
            self.reset_match()

        self.__reset_world_state()

    def __set_game_stage(self, new_stage: Referee.Stage) -> None:
        ci_input = CiInput(timestamp=int(time.time_ns()))
        api_input = Input()
        change = Change()
        change.change_stage_change.new_stage = new_stage
        api_input.change.CopyFrom(change)
        ci_input.api_inputs.append(api_input)
        self.send_ci_input(ci_input)

    def __find_unprocessed_events(
        self, referee: Referee, event_type: GameEvent.Type
    ) -> list[GameEvent]:
        """Find unprocessed game events of the specified type.

        :param referee: the referee protobuf message
        :param event_type: the type of game event to search for
        :return: list of unprocessed events of the given type
        """
        return [
            event
            for event in referee.game_events
            if event.type == event_type and event.id not in self.processed_event_ids
        ]

    def __handle_possible_goal(self, scoring_team: SslTeam) -> None:
        """Handle a possible goal event by approving the goal and resetting world state.

        :param scoring_team: the team that scored the goal
        """
        ci_input = CiInput(timestamp=int(time.time_ns()))
        api_input = Input()
        change = Change()

        # Send goal game event to resolve the possible goal
        game_event = GameEvent(
            type=GameEvent.Type.GOAL,
            origin=[
                "Majority"
            ],  # Required or else ssl-gamecontroller will convert game event to proposal
            goal=GameEvent.Goal(by_team=scoring_team),
        )

        change.add_game_event_change.game_event.CopyFrom(game_event)
        api_input.change.CopyFrom(change)
        ci_input.api_inputs.append(api_input)
        self.send_ci_input(ci_input)

        self.__reset_world_state()

    def __handle_placement_failed(self) -> None:
        """Handle a placement failed event by moving the ball to the placement point."""
        world_state = WorldState()
        world_state.ball_state.global_position.CopyFrom(
            self.latest_world.game_state.ball_placement_point
        )
        self.simulator_proto_unix_io.send_proto(WorldState, world_state)

    def __reset_world_state(self) -> None:
        """Resets the robot and ball positions"""
        self.simulator_proto_unix_io.send_proto(
            WorldState, create_default_world_state(num_robots=DIV_B_NUM_ROBOTS)
        )
        self.send_gc_command(gc_command=Command.Type.STOP, team=SslTeam.UNKNOWN)

    def __update_max_allowed_robots(self, referee: Referee) -> None:
        """Update the world state by adding or removing robots for each team to match
        their maximum allowed number of robots.

        :param referee: the referee protobuf message
        """
        world_state = WorldState()
        field_edge_y_meters: Final[int] = (
            self.latest_world.field.field_y_length
            - self.latest_world.field.boundary_buffer_size
        )

        self.__update_robot_count(
            world_state.blue_robots,
            self.latest_world.friendly_team,
            referee.blue.max_allowed_bots,
            self.blue_removed_robot_ids,
            field_edge_y_meters,
        )
        self.__update_robot_count(
            world_state.yellow_robots,
            self.latest_world.enemy_team,
            referee.yellow.max_allowed_bots,
            self.yellow_removed_robot_ids,
            -field_edge_y_meters,
        )

        # Check if we need to invert the world state
        if referee.blue_team_on_positive_half:
            for robot in itertools.chain(
                world_state.blue_robots, world_state.yellow_robots
            ):
                robot.current_state.global_position.x_meters *= -1
                robot.current_state.global_position.y_meters *= -1
                robot.current_state.global_orientation.radians += math.pi

        # Send out updated world state
        self.simulator_proto_unix_io.send_proto(WorldState, world_state)

    @staticmethod
    def __update_robot_count(
        robot_states,
        team: Team,
        max_robots: int,
        removed_robot_ids: queue.Queue[int],
        field_edge_y_meters: float,
    ) -> None:
        """Static method for updating the number of robots in the world state (i.e. robot_states) for a given Team.
        This method is team & side agnostic.

        :param robot_states: WorldState <Robot ID, RobotState> map protobuf to be updated.
        :param team: the Team of robots currently in play
        :param max_robots: The number of robots we should have on the field. Must be >= 0
        :param removed_robot_ids: The robots (IDs) which have been removed already and can safely be re-added
        :param field_edge_y_meters: Places new robots at this y position along the centerline
        :return:
        """
        # build the robot state for placing robots at the edge of field
        place_state = RobotState(
            global_position=Point(x_meters=0, y_meters=field_edge_y_meters)
        )
        # Remove robots, as we have too many. Set robot velocities to zero to avoid any drift
        for count, robot in enumerate(team.team_robots, start=1):
            if count <= max_robots:
                robot_states[robot.id].CopyFrom(robot.current_state)
                velocity = robot_states[robot.id].global_velocity
                velocity.x_component_meters = 0
                velocity.y_component_meters = 0
            else:
                removed_robot_ids.put(robot.id)
        # Add robots, since we are missing some
        robots_diff: int = max_robots - len(team.team_robots)
        if robots_diff <= 0:
            return
        for _ in range(robots_diff):
            try:
                robot_states[removed_robot_ids.get_nowait()].CopyFrom(place_state)
            except queue.Empty:
                return
