import queue
import time
import os

import pytest
import software.python_bindings as tbots
from proto.import_all_protos import *

from pyqtgraph.Qt import QtCore, QtGui

from software.networking.threaded_unix_sender import ThreadedUnixSender
from software.simulated_tests.robot_enters_region import RobotEntersRegion

from software.simulated_tests import validation
from software.thunderscope.thunderscope import Thunderscope
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.py_constants import MILLISECONDS_PER_SECOND
from software.thunderscope.binary_context_managers import (
    FullSystem,
    Simulator,
    Gamecontroller,
)
from software.thunderscope.replay.proto_logger import ProtoLogger
from proto.message_translation.tbots_protobuf import create_world_state

from software.logger.logger import createLogger
from software.simulated_tests.pytest_main import load_command_line_arguments
from software.simulated_tests.tbots_test_runner import TbotsTestRunner
from software.thunderscope.robot_communication import RobotCommunication
from software.py_constants import *
from software.simulated_tests.ball_stops_in_region import BallEventuallyStopsInRegion

logger = createLogger(__name__)

LAUNCH_DELAY_S = 0.1
WORLD_BUFFER_TIMEOUT = 5.0
PROCESS_BUFFER_DELAY_S = 0.01
PAUSE_AFTER_FAIL_DELAY_S = 3


class FieldTestRunner(TbotsTestRunner):
    """Run a simulated test"""

    def __init__(
        self,
        test_name,
        thunderscope,
        blue_full_system_proto_unix_io,
        yellow_full_system_proto_unix_io,
        gamecontroller,
    ):
        """Initialize the SimulatorTestRunner
        
        :param test_name: The name of the test to run
        :param thunderscope: The thunderscope to use, None if not used
        :param simulator_proto_unix_io: The simulator proto unix io to use
        :param blue_full_system_proto_unix_io: The blue full system proto unix io to use
        :param yellow_full_system_proto_unix_io: The yellow full system proto unix io to use
        :param gamecontroller: The gamecontroller context managed instance 

        """

        logger.info("setting up runner")

        super(FieldTestRunner, self).__init__(
            test_name,
            thunderscope,
            blue_full_system_proto_unix_io,
            yellow_full_system_proto_unix_io,
            gamecontroller,
        )

        logger.info("determining robots on field")
        # survey field for available robot ids
        try:
            world = self.world_buffer.get(block=True, timeout=WORLD_BUFFER_TIMEOUT)
            self.initial_world = world
            self.friendly_robot_ids_field = [
                robot.id for robot in world.friendly_team.team_robots
            ]
            self.enemy_robot_ids_field = [
                robot.id for robot in world.enemy_team.team_robots
            ]

            logger.info(f"blue team ids {self.friendly_robot_ids_field}")

            if len(self.friendly_robot_ids_field) == 0:
                raise Exception("no friendly robots found on field")

        except queue.Empty as empty:
            raise Exception("unable to determine robots on the field")

        logger.info("determination success")

    def set_tactics(
        self,
        tactics: AssignedTacticPlayControlParams,
        team: proto.ssl_gc_common_pb2.Team,
    ):

        if team == proto.ssl_gc_common_pb2.Team.BLUE:
            self.blue_full_system_proto_unix_io.send_proto(
                AssignedTacticPlayControlParams, tactics
            )
        else:
            self.yellow_full_system_proto_unix_io.send_proto(
                AssignedTacticPlayControlParams, tactics
            )

    def set_play(self, play: Play, team: proto.ssl_gc_common_pb2.Team):
        if team == proto.ssl_gc_common_pb2.Team.BLUE:
            self.blue_full_system_proto_unix_io.send_proto(Play, play)

        else:
            self.yellow_full_system_proto_unix_io.send_proto(Play, play)

    def send_gamecontroller_command(
        self,
        gc_command: proto.ssl_gc_state_pb2.Command,
        team: proto.ssl_gc_common_pb2.Team,
        final_ball_placement_point=None,
    ):

        self.gamecontroller.send_ci_input(
            gc_command=gc_command,
            team=team,
            final_ball_placement_point=final_ball_placement_point,
        )

    def set_worldState(self, worldstate: WorldState):
        def __set_worldState():

            BLUE_TEAM_INDEX = 0
            YELLOW_TEAM_INDEX = 1

            logger.info("testing ids matching")

            # validate that ids match with test setup
            ids_present = True
            for idx, robot_ids in enumerate(
                [worldstate.blue_robots, worldstate.yellow_robots]
            ):
                team_name = "friendly" if idx == 0 else "enemy"
                team_robots = (
                    self.friendly_robot_ids_field
                    if idx == 0
                    else self.enemy_robot_ids_field
                )
                for robot_id in robot_ids:
                    if robot_id not in team_robots:
                        logger.warning(
                            f"robot {id} from the {team_name} team present in test but not on the field"
                        )
                        ids_present = False

            if not ids_present:
                self._stop_tscope()
                raise Exception("robotIds do not match")

            logger.info("starting ball placement")

            # ball placement
            if worldstate.HasField("ball_state"):

                ball_position = tbots.createPoint(worldstate.ball_state.global_position)

                dribble_tactic = DribbleTactic(
                    dribble_destination=worldstate.ball_state.global_position,
                    allow_excessive_dribbling=True,
                )
                move_ball_tactics = AssignedTacticPlayControlParams()
                move_ball_tactics.assigned_tactics[
                    self.friendly_robot_ids_field[0]
                ].dribble.CopyFrom(dribble_tactic)
                self.blue_full_system_proto_unix_io.send_proto(
                    AssignedTacticPlayControlParams, move_ball_tactics
                )

                # validate completion
                ball_placement_timout_s = 5
                ball_placement_validation_function = BallEventuallyStopsInRegion(
                    regions=[tbots.Circle(ball_position, 0.1)]
                )

                timeout_time = time.time() + ball_placement_timout_s

                while time.time() < timeout_time:
                    try:
                        current_world = self.world_buffer.get(
                            block=True, timeout=WORLD_BUFFER_TIMEOUT
                        )

                        (
                            eventually_validation_status,
                            always_validation_status,
                        ) = validation.run_validation_sequence_sets(
                            world=current_world,
                            eventually_validation_sequence_set=[
                                [ball_placement_validation_function]
                            ],
                            always_validation_sequence_set=[[]],
                        )

                        if not validation.contains_failure(
                            eventually_validation_status
                        ):
                            logger.info("validations pass")
                            break

                    except queue.Empty as empty:
                        logger.warning("failed to obtain world")

                validation.check_validation(eventually_validation_status)

            logger.info("moving robots to position")
            # move robots to position
            initial_position_tactics = [
                AssignedTacticPlayControlParams(),
                AssignedTacticPlayControlParams(),
            ]

            robot_positions_validation_functions = [ball_placement_validation_function]

            for team_idx, team in enumerate(
                [worldstate.blue_robots, worldstate.yellow_robots]
            ):
                for robot_id in team.keys():
                    robotState = team[robot_id]
                    move_tactic = MoveTactic()
                    move_tactic.destination.CopyFrom(robotState.global_position)
                    move_tactic.final_orientation.CopyFrom(
                        robotState.global_orientation
                        if robotState.HasField("global_orientation")
                        else Angle(radians=0.0)
                    )
                    move_tactic.final_speed = 0.0
                    move_tactic.dribbler_mode = DribblerMode.OFF
                    move_tactic.ball_collision_type = BallCollisionType.AVOID
                    move_tactic.auto_chip_or_kick.CopyFrom(
                        AutoChipOrKick(autokick_speed_m_per_s=0.0)
                    )
                    move_tactic.max_allowed_speed_mode = (
                        MaxAllowedSpeedMode.PHYSICAL_LIMIT
                    )
                    move_tactic.target_spin_rev_per_s = 0.0
                    initial_position_tactics[team_idx].assigned_tactics[
                        robot_id
                    ].move.CopyFrom(move_tactic)

                    # create validation
                    expected_final_position = tbots.Point(
                        robotState.global_position.x_meters,
                        robotState.global_position.y_meters,
                    )

                    team_color = [Team.BLUE, Team.YELLOW][team_idx]
                    validation_func = SpecificRobotEventuallyEntersRegion(
                        robot_id=robot_id,
                        team=team_color,
                        regions=[tbots.Circle(expected_final_position, 0.1)],
                    )
                    robot_positions_validation_functions.append(validation_func)

            self.blue_full_system_proto_unix_io.send_proto(
                AssignedTacticPlayControlParams,
                initial_position_tactics[BLUE_TEAM_INDEX],
            )

            self.yellow_full_system_proto_unix_io.send_proto(
                AssignedTacticPlayControlParams,
                initial_position_tactics[YELLOW_TEAM_INDEX],
            )

            # validate completion
            movement_timout_s = 5
            start_time = time.time()
            timeout_time = start_time + movement_timout_s

            while time.time() < timeout_time:
                try:

                    current_world = self.world_buffer.get(
                        block=True, timeout=WORLD_BUFFER_TIMEOUT
                    )

                    (validation_status,) = validation.run_validation_sequence_sets(
                        world=current_world,
                        eventually_validation_sequence_set=[
                            robot_positions_validation_functions
                        ],
                        always_validation_sequence_set=[[]],
                    )

                    if not validation.contains_failure(validation_status):
                        break

                except queue.Empty as empty:
                    logger.warning("World Missed")

            validation.check_validation(ball_placement_validation_function)

        self._run_with_tscope(__set_worldState)

    def time_provider(self):
        """Provide the current time in seconds since the epoch"""

        with self.timestamp_mutex:
            return self.timestamp

    def run_test(
        self,
        always_validation_sequence_set=[[]],
        eventually_validation_sequence_set=[[]],
        data_loggers=[],
        test_timeout_s=3,
        tick_duration_s=0.0166,  # Default to 60hz
    ):
        """Run a test

        :param enemy_assigned_tactic_play_control_proto:
        :param assigned_tactic_play_control_proto:
        :param always_validation_sequence_set: Validation functions that should
                                hold on every tick
        :param eventually_validation_sequence_set: Validation that should
                                eventually be true, before the test ends
        :param test_timeout_s: The timeout for the test, if any eventually_validations
                                remain after the timeout, the test fails.
        :param tick_duration_s: The simulation step duration

        """

        def __runner():
            """Step simulation, full_system and run validation
            """

            start_time = time.time()
            test_end_time = time.time() + test_timeout_s

            while time.time() < test_end_time:

                # Update the timestamp logged by the ProtoLogger
                with self.timestamp_mutex:
                    ssl_wrapper = self.ssl_wrapper_buffer.get(block=False)
                    self.timestamp = ssl_wrapper.detection.t_capture

                while True:
                    try:
                        world = self.world_buffer.get(
                            block=True, timeout=WORLD_BUFFER_TIMEOUT
                        )
                        break
                    except queue.Empty as empty:
                        # If we timeout, that means full_system missed the last
                        # wrapper and robot status, lets resend it.
                        logger.warning("No world received")

                # Validate
                (
                    eventually_validation_proto_set,
                    always_validation_proto_set,
                ) = validation.run_validation_sequence_sets(
                    world,
                    eventually_validation_sequence_set,
                    always_validation_sequence_set,
                )

                # log data
                for logger in data_loggers:
                    logger.log_data(world, time_elapsed_s=time.time() - start_time)

                if self.thunderscope:
                    # Set the test name
                    eventually_validation_proto_set.test_name = self.test_name
                    always_validation_proto_set.test_name = self.test_name

                    # Send out the validation proto to thunderscope
                    self.thunderscope.blue_full_system_proto_unix_io.send_proto(
                        ValidationProtoSet, eventually_validation_proto_set
                    )
                    self.thunderscope.blue_full_system_proto_unix_io.send_proto(
                        ValidationProtoSet, always_validation_proto_set
                    )

                # Check that all always validations are always valid
                validation.check_validation(always_validation_proto_set)

            # Check that all eventually validations are eventually valid
            validation.check_validation(eventually_validation_proto_set)

            self.stop_test()

        self._run_with_tscope(__runner)


def field_test_initializer():
    args = load_command_line_arguments()

    tscope = None

    logger.info("field test initializer")
    yellow_full_system_proto_unix_io = ProtoUnixIO()
    blue_full_system_proto_unix_io = ProtoUnixIO()

    # Grab the current test name to store the proto log for the test case
    current_test = os.environ.get("PYTEST_CURRENT_TEST").split(":")[-1].split(" ")[0]
    current_test = current_test.replace("]", "")
    current_test = current_test.replace("[", "-")

    test_name = current_test.split("-")[0]

    # Launch all binaries
    with RobotCommunication(
        blue_full_system_proto_unix_io, getRobotMulticastChannel(0), args.interface
    ) as rc_blue, FullSystem(
        f"{args.blue_full_system_runtime_dir}/test/{test_name}",
        args.debug_blue_full_system,
        False,
    ) as blue_fs, FullSystem(
        f"{args.yellow_full_system_runtime_dir}/test/{test_name}",
        args.debug_yellow_full_system,
        True,
    ) as yellow_fs:
        with Gamecontroller(
            supress_logs=(not args.show_gamecontroller_logs), ci_mode=True
        ) as gamecontroller:
            print("setup all things")
            logger.info("field test initializer continues")
            blue_fs.setup_proto_unix_io(blue_full_system_proto_unix_io)
            yellow_fs.setup_proto_unix_io(yellow_full_system_proto_unix_io)

            gamecontroller.setup_proto_unix_io(
                blue_full_system_proto_unix_io, yellow_full_system_proto_unix_io,
            )

            # If we want to run thunderscope, inject the proto unix ios
            # and start the test
            if args.enable_thunderscope:
                tscope = Thunderscope(
                    blue_full_system_proto_unix_io=blue_full_system_proto_unix_io,
                    yellow_full_system_proto_unix_io=yellow_full_system_proto_unix_io,
                    layout_path=args.layout,
                    visualization_buffer_size=args.visualization_buffer_size,
                )

            logger.info("tscope initialized")

            time.sleep(LAUNCH_DELAY_S)

            # print("CALLING RUNNER")
            runner = FieldTestRunner(
                current_test,
                tscope,
                blue_full_system_proto_unix_io,
                yellow_full_system_proto_unix_io,
                gamecontroller,
            )

            logger.info("runner initialized")

            # Setup proto loggers.
            #
            # NOTE: Its important we use the test runners time provider because
            # test will run as fast as possible with a varying tick rate. The
            # SimulatorTestRunner time provider is tied to the simulators
            # t_capture coming out of the wrapper packet (rather than time.time).
            with ProtoLogger(
                f"{args.blue_full_system_runtime_dir}/logs/{current_test}",
                time_provider=runner.time_provider,
            ) as blue_logger, ProtoLogger(
                f"{args.yellow_full_system_runtime_dir}/logs/{current_test}",
                time_provider=runner.time_provider,
            ) as yellow_logger:
                blue_full_system_proto_unix_io.register_to_observe_everything(
                    blue_logger.buffer
                )
                yellow_full_system_proto_unix_io.register_to_observe_everything(
                    yellow_logger.buffer
                )
                logger.info("yielding")
                yield runner
                print(
                    f"\n\nTo replay this test for the blue team, go to the `src` folder and run \n./tbots.py run thunderscope --blue_log {blue_logger.log_folder}"
                )
                print(
                    f"\n\nTo replay this test for the yellow team, go to the `src` folder and run \n./tbots.py run thunderscope --yellow_log {yellow_logger.log_folder}"
                )


@pytest.fixture
def field_test_runner():
    initializer = field_test_initializer()

    yield next(initializer)

    # test teardown
    try:
        next(initializer)
    except StopIteration:
        pass
