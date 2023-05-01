import pdb
import queue
import time
import os
import threading

import pytest
import software.python_bindings as tbots
import argparse
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
# from software.simulated_tests.pytest_main import load_command_line_arguments
from software.simulated_tests.tbots_test_runner import TbotsTestRunner
from software.thunderscope.robot_communication import RobotCommunication
from software.py_constants import *
from software.simulated_tests.ball_stops_in_region import BallEventuallyStopsInRegion
import faulthandler

logger = createLogger(__name__)

WORLD_BUFFER_TIMEOUT = 5.0
PROCESS_BUFFER_DELAY_S = 0.01
PAUSE_AFTER_FAIL_DELAY_S = 3
LAUNCH_DELAY_S = 0.1
TEST_END_DELAY = 0.3


class FieldTestRunner(TbotsTestRunner):
    """Run a field test"""

    def __init__(
        self,
        test_name,
        thunderscope,
        blue_full_system_proto_unix_io,
        yellow_full_system_proto_unix_io,
        gamecontroller,
        publish_validation_protos=True,
    ):
        """Initialize the FieldTestRunner
        
        :param test_name: The name of the test to run
        :param blue_full_system_proto_unix_io: The blue full system proto unix io to use
        :param yellow_full_system_proto_unix_io: The yellow full system proto unix io to use
        :param gamecontroller: The gamecontroller context managed instance 
        :param publish_validation_protos: whether to publish validation protos

        """

        super(FieldTestRunner, self).__init__(
            test_name,
            thunderscope,
            blue_full_system_proto_unix_io,
            yellow_full_system_proto_unix_io,
            gamecontroller,
        )
        # pdb.set_trace()
        print("A", flush=True)
        self.publish_validation_protos = publish_validation_protos

        logger.info("determining robots on field")
        # survey field for available robot ids
        try:
            # print("field_test_fixture.py line 83: world_buffer.get",flush=True)
            world = self.world_buffer.get(block=True, timeout=WORLD_BUFFER_TIMEOUT)
            logger.info(world)
            self.initial_world = world
            self.friendly_robot_ids_field = [
                robot.id for robot in world.friendly_team.team_robots
            ]
            self.enemy_robot_ids_field = [
                robot.id for robot in world.enemy_team.team_robots
            ]

            logger.info(f"blue team ids {self.friendly_robot_ids_field}")

            if len(self.friendly_robot_ids_field) == 0:
                print("raising an exception",flush=True)
                raise Exception("no friendly robots found on field")

        except queue.Empty as empty:
            print("empty",flush=True)
            raise Exception("unable to determine robots on the field")

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
        """ Sets a world state on the field by moving robots and the ball to the given positions

        Args:
            worldstate (WorldState): Proto which contains robot and ball positions expected on the field

        Raises:
            Exception: On failing to verify that the expected position was set
        """

        self.__validateMatchingRobotIds(worldstate)
        self.__setBallState(worldstate)
        self.__setRobotState(worldstate)

    def __validateWorldState(self, eventually_validation_set, timeout):
        timeout_time = time.time() + timeout

        while time.time() < timeout_time:
            try:

                #print("field_test_fixture.py line 137: world_buffer.get",flush=True)
                current_world = self.world_buffer.get(
                    block=True, timeout=WORLD_BUFFER_TIMEOUT
                )

                (
                    eventually_validation_status,
                    always_validation_status,
                ) = validation.run_validation_sequence_sets(
                    world=current_world,
                    eventually_validation_sequence_set=[[eventually_validation_set]],
                    always_validation_sequence_set=[[]],
                )

                if not validation.contains_failure(eventually_validation_status):
                    break

            except queue.Empty as empty:
                logger.warning("failed to obtain world")

        validation.check_validation(eventually_validation_status)

    def __validateMatchingRobotIds(self, worldstate: WorldState):
        """Checks whether the robots to be used in this field test matches up with the robots we see on the field

        Args:
            worldstate (worldState Proto): The worldstate containing the robots to be used in the test

        Raises:
            Exception: On there existing a robot to be used in the test but is not on the field
        """
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
            raise Exception("robotIds do not match")

    def __setBallState(self, worldstate: WorldState):
        """Commands a robot to move the ball to the given position on the field

        Args:
            worldstate (worldState Proto): proto that contains ball position

        Raises:
            Exception: On failing to move the ball to the correct position
        """

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
            ball_placement_timout_s = 10
            ball_placement_validation_function = BallEventuallyStopsInRegion(
                regions=[tbots.Circle(ball_position, 0.1)]
            )

            try:
                self.__validateWorldState(
                    ball_placement_validation_function, ball_placement_timout_s
                )
            except:
                raise Exception(
                    "ball placement by blue robot {} to position {} failed".format(
                        self.friendly_robot_ids_field[0], ball_position
                    )
                )

    def __setRobotState(self, worldstate: WorldState):
        """sets friendly and enemy robots to the given position on the field

        Args:
            worldstate (worldState Proto): proto that contains robot positions

        Raises:
            Exception: On failing to set robot positions
        """
        logger.info("moving robots to start position")

        BLUE_TEAM_INDEX = 0
        YELLOW_TEAM_INDEX = 1

        # creating movement tactics and associated validation functions
        initial_position_tactics = [
            AssignedTacticPlayControlParams(),
            AssignedTacticPlayControlParams(),
        ]

        robot_positions_validation_functions = []

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
            AssignedTacticPlayControlParams, initial_position_tactics[BLUE_TEAM_INDEX],
        )

        self.yellow_full_system_proto_unix_io.send_proto(
            AssignedTacticPlayControlParams,
            initial_position_tactics[YELLOW_TEAM_INDEX],
        )

        # validate completion
        movement_timout_s = 10
        try:
            self.__validateWorldState(
                robot_positions_validation_functions, movement_timout_s
            )
        except:
            raise Exception("robot positioning not set")

    def time_provider(self):
        """Provide the current time in seconds since the epoch"""

        with self.timestamp_mutex:
            return self.timestamp

    def run_test(
        self,
        always_validation_sequence_set=[[]],
        eventually_validation_sequence_set=[[]],
        test_timeout_s=3,
    ):
        """Run a test. In a field test this means beginning validation.

        :param always_validation_sequence_set: Validation functions that should
                                hold on every tick
        :param eventually_validation_sequence_set: Validation that should
                                eventually be true, before the test ends
        :param test_timeout_s: The timeout for the test, if any eventually_validations
                                remain after the timeout, the test fails.
        """
        print("\n\n\nIN RUN_TEST\n\n\n",flush=True)
        if(self.thunderscope):
            print("We have Thunderscope",flush=True)
        else:
            print("We do not have Thunderscope", flush=True)

        def stop_test(delay):
            print("stop_test",flush=True)
            time.sleep(delay)
            if self.thunderscope:
                self.thunderscope.close()
            print("test is over", flush=True)

        # class Excepthook(object):
        #     def __init__(self):
        #         self.last_exception = None
        #
        #     def excepthook(self, args):
        #         """This function is _critical_ for show_thunderscope to work.
        #     If the test Thread raises an exception we won't be able to close
        #     the window from the main thread.
        #
        #     :param args: The args passed in from the hook
        #
        #     """
        #         stop_test(PAUSE_AFTER_FAIL_DELAY_S)
        #         self.last_exception = args.exc_value
        #         print(args.exc_value,flush=True)
        #         raise self.last_exception

        def __runner():
            time.sleep(LAUNCH_DELAY_S)

            test_end_time = time.time() + test_timeout_s

            while time.time() < test_end_time:
                # Update the timestamp logged by the ProtoLogger
                with self.timestamp_mutex:

                    #print("field_test_fixture.py line 369: ssl_wrapper_buffer.get",flush=True)
                    ssl_wrapper = self.ssl_wrapper_buffer.get(block=False)
                    self.timestamp = ssl_wrapper.detection.t_capture

                while True:
                    try:
                        # print("field_test_fixture.py line 369: world_buffer.get",flush=True)
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

                if self.publish_validation_protos:
                    # Set the test name
                    eventually_validation_proto_set.test_name = self.test_name
                    always_validation_proto_set.test_name = self.test_name

                    # Send out the validation proto to thunderscope
                    self.blue_full_system_proto_unix_io.send_proto(
                        ValidationProtoSet, eventually_validation_proto_set
                    )
                    self.blue_full_system_proto_unix_io.send_proto(
                        ValidationProtoSet, always_validation_proto_set
                    )

                # Check that all always validations are always valid
                validation.check_validation(always_validation_proto_set)

            # Check that all eventually validations are eventually valid
            validation.check_validation(eventually_validation_proto_set)
            stop_test(TEST_END_DELAY)


        # ex = Excepthook()
        def excepthook(args):
            """This function is _critical_ for show_thunderscope to work.
            If the test Thread will raises an exception we won't be able to close
            the window from the main thread.

            :param args: The args passed in from the hook

            """

            stop_test(delay=PAUSE_AFTER_FAIL_DELAY_S)
            self.last_exception = args.exc_value
            raise self.last_exception

        threading.excepthook = excepthook

        if self.thunderscope:
            run_sim_thread = threading.Thread(target=__runner, daemon=True)
            run_sim_thread.start()
            self.thunderscope.show()
            print("Showing TSCOPE",flush=True)
            run_sim_thread.join()

            if self.last_exception:
                print("exception raised if thunderscope",flush=True)
                pytest.fail(str(ex.last_exception))

        else:
            __runner()


# def field_test_initializer(
#     simulator_proto_unix_io,yellow_full_system_proto_unix_io, blue_full_system_proto_unix_io
# ):
#     args = load_command_line_arguments()
#
#     # Grab the current test name to store the proto log for the test case
#     current_test = os.environ.get("PYTEST_CURRENT_TEST").split(":")[-1].split(" ")[0]
#     current_test = current_test.replace("]", "")
#     current_test = current_test.replace("[", "-")
#
#     test_name = current_test.split("-")[0]
#     tscope = None
#
#     # Launch all binaries
#     with FullSystem(
#         f"{args.blue_full_system_runtime_dir}/test/{test_name}",
#         debug_full_system=args.debug_blue_full_system,
#         friendly_colour_yellow=False,
#         should_restart_on_crash=False,
#     ) as blue_fs, RobotCommunication(
#         current_proto_unix_io=blue_full_system_proto_unix_io,
#         multicast_channel=getRobotMulticastChannel(0),
#         interface=args.interface,
#         disable_estop=False
#     ) as rc_blue, FullSystem(
#         f"{args.yellow_full_system_runtime_dir}/test/{test_name}",
#         debug_full_system=args.debug_yellow_full_system,
#         friendly_colour_yellow=True,
#         should_restart_on_crash=False,
#     ) as yellow_fs:
#         with Gamecontroller(
#             supress_logs=(not args.show_gamecontroller_logs), ci_mode=True
#         ) as gamecontroller:
#             blue_fs.setup_proto_unix_io(blue_full_system_proto_unix_io)
#             yellow_fs.setup_proto_unix_io(yellow_full_system_proto_unix_io)
#             rc_blue.setup_for_fullsystem()
#
#             gamecontroller.setup_proto_unix_io(
#                 blue_full_system_proto_unix_io, yellow_full_system_proto_unix_io,
#             )
#             print("opened GC", flush=True)
#             # If we want to run thunderscope, inject the proto unix ios
#             # and start the test
#             if args.enable_thunderscope:
#                 print("tscope enabled", flush=True)
#                 tscope = Thunderscope(
#                     simulator_proto_unix_io=simulator_proto_unix_io,
#                     blue_full_system_proto_unix_io=blue_full_system_proto_unix_io,
#                     yellow_full_system_proto_unix_io=yellow_full_system_proto_unix_io,
#                     # layout_path=args.layout,
#                     layout_path=None,
#                     visualization_buffer_size=args.visualization_buffer_size,
#                     load_blue=True,
#                     load_yellow=False
#                 )
#                 print("pre-E", flush=True)
#             print("E", flush=True)
#             runner = FieldTestRunner(
#                 test_name=current_test,
#                 blue_full_system_proto_unix_io=blue_full_system_proto_unix_io,
#                 yellow_full_system_proto_unix_io=yellow_full_system_proto_unix_io,
#                 gamecontroller=gamecontroller,
#                 thunderscope=tscope
#             )
#             print("F", flush=True)
#
#             # Setup proto loggers.
#             #
#             # NOTE: Its important we use the test runners time provider because
#             # test will run as fast as possible with a varying tick rate. The
#             # SimulatorTestRunner time provider is tied to the simulators
#             # t_capture coming out of the wrapper packet (rather than time.time).
#             with ProtoLogger(
#                 f"{args.blue_full_system_runtime_dir}/logs/{current_test}",
#                 time_provider=runner.time_provider,
#             ) as blue_logger, ProtoLogger(
#                 f"{args.yellow_full_system_runtime_dir}/logs/{current_test}",
#                 time_provider=runner.time_provider,
#             ) as yellow_logger:
#                 blue_full_system_proto_unix_io.register_to_observe_everything(
#                     blue_logger.buffer
#                 )
#                 yellow_full_system_proto_unix_io.register_to_observe_everything(
#                     yellow_logger.buffer
#                 )
#                 print("Z",flush=True)
#                 yield runner
#                 print(
#                     f"\n\nTo replay this test for the blue team, go to the `src` folder and run \n./tbots.py run thunderscope --blue_log {blue_logger.log_folder}", flush=True
#                 )
#                 print(
#                     f"\n\nTo replay this test for the yellow team, go to the `src` folder and run \n./tbots.py run thunderscope --yellow_log {yellow_logger.log_folder}", flush=True
#                 )
#     return


def load_command_line_arguments():
    """Load from command line arguments using argpase
    NOTE: Pytest has its own built in argument parser (conftest.py, pytest_addoption)
    but it doesn't seem to play nicely with bazel. We just use argparse instead.
    """
    parser = argparse.ArgumentParser(description="Run simulated or field pytests")
    parser.add_argument(
        "--enable_thunderscope", action="store_true", help="enable thunderscope"
    )
    parser.add_argument(
        "--simulator_runtime_dir",
        type=str,
        help="simulator runtime directory",
        default="/tmp/tbots",
    )
    parser.add_argument(
        "--blue_full_system_runtime_dir",
        type=str,
        help="blue full_system runtime directory",
        default="/tmp/tbots/blue",
    )
    parser.add_argument(
        "--yellow_full_system_runtime_dir",
        type=str,
        help="yellow full_system runtime directory",
        default="/tmp/tbots/yellow",
    )
    parser.add_argument(
        "--layout",
        action="store",
        help="Which layout to run, if not specified the last layout will run",
    )
    parser.add_argument(
        "--debug_blue_full_system",
        action="store_true",
        default=False,
        help="Debug blue full_system",
    )
    parser.add_argument(
        "--debug_yellow_full_system",
        action="store_true",
        default=False,
        help="Debug yellow full_system",
    )
    parser.add_argument(
        "--debug_simulator",
        action="store_true",
        default=False,
        help="Debug the simulator",
    )
    parser.add_argument(
        "--visualization_buffer_size",
        action="store",
        type=int,
        default=5,
        help="How many packets to buffer while rendering",
    )
    parser.add_argument(
        "--show_gamecontroller_logs",
        action="store_true",
        default=False,
        help="How many packets to buffer while rendering",
    )
    parser.add_argument(
        "--run_field_test",
        action="store_true",
        default=False,
        help="whether to run test as a field test instead of a simulated test",
    )
    parser.add_argument(
        "--test_filter",
        action="store",
        default="",
        help="The test filter, if not specified all tests will run. "
             + "See https://docs.pytest.org/en/latest/how-to/usage.html#specifying-tests-selecting-tests",
    )

    parser.add_argument(
        "--interface",
        action="store",
        type=str,
        default=None,
        help="Which interface to communicate over",
    )

    parser.add_argument(
        "--estop_path",
        action="store",
        type=str,
        default="/dev/ttyACM0",
        help="Path to the Estop",
    )

    parser.add_argument(
        "--estop_baudrate",
        action="store",
        type=int,
        default=115200,
        help="Estop Baudrate",
    )

    return parser.parse_args()

@pytest.fixture
def field_test_runner():
    print("sleepin'", flush=True)
    time.sleep(0)
    faulthandler.enable()
    print("C", flush=True)
    simulator_proto_unix_io = ProtoUnixIO()
    yellow_full_system_proto_unix_io = ProtoUnixIO()
    blue_full_system_proto_unix_io = ProtoUnixIO()
    args = load_command_line_arguments()

    # Grab the current test name to store the proto log for the test case
    current_test = os.environ.get("PYTEST_CURRENT_TEST").split(":")[-1].split(" ")[0]
    current_test = current_test.replace("]", "")
    current_test = current_test.replace("[", "-")

    test_name = current_test.split("-")[0]
    tscope = None

    # Launch all binaries
    with FullSystem(
        f"{args.blue_full_system_runtime_dir}/test/{test_name}",
        debug_full_system=args.debug_blue_full_system,
        friendly_colour_yellow=False,
        should_restart_on_crash=False,
    ) as blue_fs, RobotCommunication(
        current_proto_unix_io=blue_full_system_proto_unix_io,
        multicast_channel=getRobotMulticastChannel(0),
        interface=args.interface,
        disable_estop=False
    ) as rc_blue, FullSystem(
        f"{args.yellow_full_system_runtime_dir}/test/{test_name}",
        debug_full_system=args.debug_yellow_full_system,
        friendly_colour_yellow=True,
        should_restart_on_crash=False,
    ) as yellow_fs:
        with Gamecontroller(
            supress_logs=(not args.show_gamecontroller_logs), ci_mode=True
        ) as gamecontroller:
            blue_fs.setup_proto_unix_io(blue_full_system_proto_unix_io)
            yellow_fs.setup_proto_unix_io(yellow_full_system_proto_unix_io)
            rc_blue.setup_for_fullsystem()

            gamecontroller.setup_proto_unix_io(
                blue_full_system_proto_unix_io, yellow_full_system_proto_unix_io,
            )
            # If we want to run thunderscope, inject the proto unix ios
            # and start the test
            if args.enable_thunderscope:
                print("tscope enabled", flush=True)
                tscope = Thunderscope(
                    simulator_proto_unix_io=simulator_proto_unix_io,
                    blue_full_system_proto_unix_io=blue_full_system_proto_unix_io,
                    yellow_full_system_proto_unix_io=yellow_full_system_proto_unix_io,
                    # layout_path=args.layout,
                    layout_path=None,
                    visualization_buffer_size=args.visualization_buffer_size,
                    load_blue=True,
                    load_yellow=False
                )
                print("pre-E", flush=True)
            print("E", flush=True)
            time.sleep(LAUNCH_DELAY_S)
            runner = FieldTestRunner(
                test_name=current_test,
                blue_full_system_proto_unix_io=blue_full_system_proto_unix_io,
                yellow_full_system_proto_unix_io=yellow_full_system_proto_unix_io,
                gamecontroller=gamecontroller,
                thunderscope=tscope
            )
            blue_full_system_proto_unix_io.register_observer(World,runner.world_buffer)
            print("F", flush=True)

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
                print("Z",flush=True)
                yield runner
                rc_blue.robots_connected_to_fullsystem = None
                print(
                    f"\n\nTo replay this test for the blue team, go to the `src` folder and run \n./tbots.py run thunderscope --blue_log {blue_logger.log_folder}", flush=True
                )
                print(
                    f"\n\nTo replay this test for the yellow team, go to the `src` folder and run \n./tbots.py run thunderscope --yellow_log {yellow_logger.log_folder}", flush=True
                )
    print("exiting field test runner fixture",flush=True)
    # initializer = field_test_initializer(
    #     simulator_proto_unix_io=simulator_proto_unix_io,
    #     blue_full_system_proto_unix_io=blue_full_system_proto_unix_io,
    #     yellow_full_system_proto_unix_io=yellow_full_system_proto_unix_io,
    # )

    # print("running first next in runner",flush=True)
    # yield_val = next(initializer)
    # yield yield_val
    # print("after yield in runner",flush=True)
    # test teardown
    # try:
    #     print("running second next in runner",flush=True)
    #     next(initializer)
    # except StopIteration as e:
    #     # raise e
    #     pass
    # yield