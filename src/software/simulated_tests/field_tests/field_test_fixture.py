import threading
import queue
import argparse
import time
import sys
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
from proto.message_translation.tbots_protobuf import create_world_state, parse_world_state

from software.logger.logger import createLogger
from software.simulated_tests.pytest_main import load_command_line_arguments

logger = createLogger(__name__)

LAUNCH_DELAY_S = 0.1
WORLD_BUFFER_TIMEOUT = 0.5
PROCESS_BUFFER_DELAY_S = 0.01
PAUSE_AFTER_FAIL_DELAY_S = 3


class FieldTestRunner(object):

    """Run a simulated test"""

    def __init__(
        self,
        test_name,
        thunderscope,
        simulator_proto_unix_io,
        blue_full_system_proto_unix_io,
        yellow_full_system_proto_unix_io,
        gamecontroller,
        initial_worldstate
    ):
        """Initialize the SimulatorTestRunner
        
        :param test_name: The name of the test to run
        :param thunderscope: The thunderscope to use, None if not used
        :param simulator_proto_unix_io: The simulator proto unix io to use
        :param blue_full_system_proto_unix_io: The blue full system proto unix io to use
        :param yellow_full_system_proto_unix_io: The yellow full system proto unix io to use
        :param gamecontroller: The gamecontroller context managed instance 

        """

        self.test_name = test_name
        self.thunderscope = thunderscope
        self.simulator_proto_unix_io = simulator_proto_unix_io
        self.blue_full_system_proto_unix_io = blue_full_system_proto_unix_io
        self.yellow_full_system_proto_unix_io = yellow_full_system_proto_unix_io
        self.gamecontroller = gamecontroller
        self.last_exception = None

        self.world_buffer = ThreadSafeBuffer(buffer_size=1, protobuf_type=World)
        self.last_exception = None
        self.initial_worldstate = initial_worldstate

        self.ssl_wrapper_buffer = ThreadSafeBuffer(
            buffer_size=1, protobuf_type=SSL_WrapperPacket
        )
        self.robot_status_buffer = ThreadSafeBuffer(
            buffer_size=1, protobuf_type=RobotStatus
        )

        self.blue_full_system_proto_unix_io.register_observer(
            SSL_WrapperPacket, self.ssl_wrapper_buffer
        )
        self.blue_full_system_proto_unix_io.register_observer(
            RobotStatus, self.robot_status_buffer
        )

        # Only validate on the blue worlds
        self.blue_full_system_proto_unix_io.register_observer(World, self.world_buffer)

        tick = SimulatorTick(
            milliseconds=0.0166 * MILLISECONDS_PER_SECOND
        )
        simulator_proto_unix_io.send_proto(SimulatorTick, tick)


        self.timestamp = 0
        self.timestamp_mutex = threading.Lock()

        #self.thunderscope.show()

        # todo possibly clear buffer
        #self.world_buffer.clear()

        #survey field for robot ids
        try:
            world = self.world_buffer.get(
                block=True, timeout=WORLD_BUFFER_TIMEOUT
            )
            print("WORLS RETURNED", world)
            self.friendly_robot_ids_field = [ robot.id for robot in world.friendly_team.team_robots ]
            self.enemy_robot_ids_field = [ robot.id for robot in world.enemy_team.team_robots ]
        except queue.Empty as empty:
            print("unable to determine robots on the field (print stment")
            logger.fatal("unable to determine robots on the field")

        #validate that ids match with test setup
        ids_present = True
        for idx, robot_ids in enumerate([self.friendly_robot_ids_field, self.enemy_robot_ids_field]):
            team_name = 'friendly' if idx ==0 else 'enemy'
            team_robots = initial_worldstate.blue_robots if idx == 0 else initial_worldstate.yellow_robots
            for robot_id in robot_ids:
                if robot_id not in team_robots.keys():
                    logger.warning(f"robot {id} from the {team_name} team present in test but not on the field")
                    ids_present = False

        if not ids_present:
            logger.fatal("could not continue test - robotIds do not match")

        self.set_worldState(initial_worldstate)


    def set_worldState(self, worldstate : WorldState):

        #ball placement
        if worldstate.HasField('ball_state'):

            ball_position = tbots.createPoint(worldstate.ball_state.global_position)

            dribble_tactic = DribbleTactic(dribble_destination = worldstate.ball_state.global_position, allow_excessive_dribbling=True )
            #dribble_tactic.dribble.CopyFrom(DribbleTactic(dribble_destination = worldstate.ball_state.global_position, allow_excessive_dribbling=True ))
            print(self.friendly_robot_ids_field)
            move_ball_tactics = AssignedTacticPlayControlParams()
            move_ball_tactics.assigned_tactics[self.friendly_robot_ids_field[0]].dribble.CopyFrom(dribble_tactic)
            logger.info(move_ball_tactics)
            print("sending dribble at t", time.time())
            self.blue_full_system_proto_unix_io.send_proto(
                AssignedTacticPlayControlParams, move_ball_tactics
            )
            logger.info("sent dribble")
            # validate completion
            ball_placement_timout_s = 5
            start_time = time.time()
            timeout_time = start_time + ball_placement_timout_s

            # self.world_buffer.clear()
            time.sleep(3)

            while time.time() < timeout_time:
                tick = SimulatorTick(
                    milliseconds=0.0166 * MILLISECONDS_PER_SECOND
                )
                self.simulator_proto_unix_io.send_proto(SimulatorTick, tick)

                try:
                    current_world = self.world_buffer.get(
                        block=True, timeout=WORLD_BUFFER_TIMEOUT
                    )
                    if (ball_position - tbots.createPoint(current_world.ball.current_state.global_position)).length() < 0.1:
                        print("successfully moved ball")
                        logger.info("successfully moved ball")
                        break

                except queue.Empty as empty:
                    # If we timeout, that means full_system missed the last
                    # wrapper and robot status, lets resend it.
                    logger.warning("Fullsystem missed last wrapper, resending ...")

                    ssl_wrapper = self.ssl_wrapper_buffer.get(block=False)
                    robot_status = self.robot_status_buffer.get(block=False)

                    self.blue_full_system_proto_unix_io.send_proto(
                        SSL_WrapperPacket, ssl_wrapper
                    )
                    self.blue_full_system_proto_unix_io.send_proto(
                        RobotStatus, robot_status
                    )

            if time.time() >= timeout_time:
                logger.fatal("unable to place ball in correct position")


        #move robots to position
        initial_position_tactics = [AssignedTacticPlayControlParams(), AssignedTacticPlayControlParams()]

        for team_idx, team in enumerate([worldstate.blue_robots, worldstate.yellow_robots]):
            for robot_id in team.keys():
                robotState = team[robot_id]
                move_tactic = MoveTactic()
                move_tactic.destination.CopyFrom(robotState.global_position)
                print("in python creating move with destination ", robotState.global_position.x_meters)
                move_tactic.final_orientation.CopyFrom(robotState.global_orientation if robotState.HasField('global_orientation') else Angle(radians=0.0))
                move_tactic.final_speed = 0.0
                move_tactic.dribbler_mode = DribblerMode.OFF
                move_tactic.ball_collision_type = BallCollisionType.AVOID
                move_tactic.auto_chip_or_kick.CopyFrom(AutoChipOrKick(autokick_speed_m_per_s=6.0))
                move_tactic.max_allowed_speed_mode = MaxAllowedSpeedMode.PHYSICAL_LIMIT
                move_tactic.target_spin_rev_per_s = 0.0
                initial_position_tactics[team_idx].assigned_tactics[robot_id].move.CopyFrom(move_tactic)

        print("sending movement at t", time.time())

        self.blue_full_system_proto_unix_io.send_proto(
            AssignedTacticPlayControlParams, initial_position_tactics[0]
        )

        self.yellow_full_system_proto_unix_io.send_proto(
            AssignedTacticPlayControlParams, initial_position_tactics[1]
        )

        #validate completion
        ball_placement_timout_s = 10
        start_time = time.time()
        timeout_time = start_time + ball_placement_timout_s
        self.world_buffer.clear()

        completed = False
        while time.time() < timeout_time and not completed:
            try:
                current_world = self.world_buffer.get(
                    block=True, timeout=WORLD_BUFFER_TIMEOUT
                )

                for team_idx, team in enumerate([worldstate.blue_robots, worldstate.yellow_robots]):
                    for robot_id in team.keys():
                        robotState = team[robot_id]

                for robot_id in worldstate.blue_robots.keys():
                    expected_position = tbots.createPoint(worldstate.blue_robots[robot_id].global_position)
                    robot = next((robot_ for robot_ in current_world.friendly_team.team_robots if robot_.id == robot_id), None)

                    if robot is None:
                        logger.warning(f"robot with id {robot_id} missing from world")
                        continue

                    current_position = tbots.Point(robot.current_state.global_position.x_meters, robot.current_state.global_position.y_meters)

                    print(expected_position, current_position, (expected_position - current_position).length())
                    if (expected_position - current_position).length() > 0.1:
                        completed = False

                for robot_id in worldstate.yellow_robots.keys():
                    expected_position = tbots.createPoint(worldstate.yellow_robots[robot_id].global_position)
                    robot = next((robot_ for robot_ in current_world.enemy_team.team_robots if robot_.id == robot_id), None)

                    if robot is None:
                        logger.warning(f"robot with id {robot_id} missing from world")
                        continue

                    current_position = tbots.Point(robot.current_state.global_position.x_meters, robot.current_state.global_position.y_meters)

                    if (expected_position - current_position).length() > 0.1:
                        continue



            except queue.Empty as empty:
                # If we timeout, that means full_system missed the last
                # wrapper and robot status, lets resend it.
                logger.warning("Fullsystem missed last wrapper, resending ...")

                ssl_wrapper = self.ssl_wrapper_buffer.get(block=False)
                robot_status = self.robot_status_buffer.get(block=False)

                self.blue_full_system_proto_unix_io.send_proto(
                    SSL_WrapperPacket, ssl_wrapper
                )
                self.blue_full_system_proto_unix_io.send_proto(
                    RobotStatus, robot_status
                )


        if time.time() >= timeout_time:
            logger.fatal("unable to place robots in correct positions")

        logger.info("successfully moved robot")



    def time_provider(self):
        """Provide the current time in seconds since the epoch"""

        with self.timestamp_mutex:
            return self.timestamp

    def run_test(
        self,
        always_validation_sequence_set=[[]],
        eventually_validation_sequence_set=[[]],
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


        def __stopper(delay=PROCESS_BUFFER_DELAY_S):
            """Stop running the test

            :param delay: How long to wait before closing everything, defaults
                          to PROCESS_BUFFER_DELAY_S to minimize buffer warnings

            """
            time.sleep(delay)

            if self.thunderscope:
                self.thunderscope.close()

        def __runner():
            """Step simulation, full_system and run validation
            """

            print("__runner called at time", time.time())
            logger.info("starting test")
            time_elapsed_s = 0
            #self.set_worldState(self.initial_worldstate)

            while time_elapsed_s < test_timeout_s:

                # Update the timestamp logged by the ProtoLogger
                with self.timestamp_mutex:
                    ssl_wrapper = self.ssl_wrapper_buffer.get(block=False)
                    self.timestamp = ssl_wrapper.detection.t_capture

                tick = SimulatorTick(
                    milliseconds=tick_duration_s * MILLISECONDS_PER_SECOND
                )
                self.simulator_proto_unix_io.send_proto(SimulatorTick, tick)
                time_elapsed_s += tick_duration_s

                if self.thunderscope:
                    time.sleep(tick_duration_s)

                while True:
                    try:
                        world = self.world_buffer.get(
                            block=True, timeout=WORLD_BUFFER_TIMEOUT
                        )
                        break
                    except queue.Empty as empty:
                        # If we timeout, that means full_system missed the last
                        # wrapper and robot status, lets resend it.
                        logger.warning("Fullsystem missed last wrapper, resending ...")

                        ssl_wrapper = self.ssl_wrapper_buffer.get(block=False)
                        robot_status = self.robot_status_buffer.get(block=False)

                        self.blue_full_system_proto_unix_io.send_proto(
                            SSL_WrapperPacket, ssl_wrapper
                        )
                        self.blue_full_system_proto_unix_io.send_proto(
                            RobotStatus, robot_status
                        )

                # Validate
                (
                    eventually_validation_proto_set,
                    always_validation_proto_set,
                ) = validation.run_validation_sequence_sets(
                    world,
                    eventually_validation_sequence_set,
                    always_validation_sequence_set,
                )

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

            __stopper()

        def excepthook(args):
            """This function is _critical_ for show_thunderscope to work.
            If the test Thread will raises an exception we won't be able to close
            the window from the main thread.

            :param args: The args passed in from the hook

            """

            __stopper(delay=PAUSE_AFTER_FAIL_DELAY_S)
            self.last_exception = args.exc_value
            raise self.last_exception

        threading.excepthook = excepthook

        # If thunderscope is enabled, run the test in a thread and show
        # thunderscope on this thread. The excepthook is setup to catch
        # any test failures and propagate them to the main thread
        if self.thunderscope:

            run_sim_thread = threading.Thread(target=__runner, daemon=True)
            run_sim_thread.start()
            self.thunderscope.show()
            run_sim_thread.join()

            if self.last_exception:
                pytest.fail(str(self.last_exception))

        # If thunderscope is disabled, just run the test
        else:
            __runner()

@pytest.fixture
def field_test_runner(request):
    args = load_command_line_arguments()

    tscope = None

    simulator_proto_unix_io = ProtoUnixIO()
    yellow_full_system_proto_unix_io = ProtoUnixIO()
    blue_full_system_proto_unix_io = ProtoUnixIO()

    # Grab the current test name to store the proto log for the test case
    current_test = os.environ.get("PYTEST_CURRENT_TEST").split(":")[-1].split(" ")[0]
    current_test = current_test.replace("]", "")
    current_test = current_test.replace("[", "-")

    test_name = current_test.split("-")[0]

    # Launch all binaries
    with Simulator(
        f"{args.simulator_runtime_dir}/test/{test_name}", args.debug_simulator
    ) as simulator, FullSystem(
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

            blue_fs.setup_proto_unix_io(blue_full_system_proto_unix_io)
            yellow_fs.setup_proto_unix_io(yellow_full_system_proto_unix_io)
            simulator.setup_proto_unix_io(
                simulator_proto_unix_io,
                blue_full_system_proto_unix_io,
                yellow_full_system_proto_unix_io,
            )
            gamecontroller.setup_proto_unix_io(
                blue_full_system_proto_unix_io, yellow_full_system_proto_unix_io,
            )

            # If we want to run thunderscope, inject the proto unix ios
            # and start the test
            if args.enable_thunderscope:
                tscope = Thunderscope(
                    simulator_proto_unix_io,
                    blue_full_system_proto_unix_io,
                    yellow_full_system_proto_unix_io,
                    layout_path=args.layout,
                    visualization_buffer_size=args.visualization_buffer_size,
                )

            time.sleep(LAUNCH_DELAY_S)

            #todo remove when robotcommunication is merged
            ball_initial_position = tbots.Point(2.5, 0)
            kick_velocity = tbots.Vector(2, 0)

            rob_pos = ball_initial_position - (kick_velocity.normalize() * 1.5)

            # Setup Ball
            simulator_proto_unix_io.send_proto(
                WorldState,
                create_world_state(
                    [],
                    blue_robot_locations=[rob_pos],
                    ball_location=ball_initial_position,
                    ball_velocity=tbots.Vector(0, 0),
                ),
            )

            time.sleep(1)
            #print("CALLING RUNNER")
            runner = FieldTestRunner(
                current_test,
                tscope,
                simulator_proto_unix_io,
                blue_full_system_proto_unix_io,
                yellow_full_system_proto_unix_io,
                gamecontroller,
                request.param
            )

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
                logger.info("yielding runner")
                print("YIELDING RUNNER")

                yield runner
                print(
                    f"\n\nTo replay this test for the blue team, go to the `src` folder and run \n./tbots.py run thunderscope --blue_log {blue_logger.log_folder}"
                )
                print(
                    f"\n\nTo replay this test for the yellow team, go to the `src` folder and run \n./tbots.py run thunderscope --yellow_log {yellow_logger.log_folder}"
                )
