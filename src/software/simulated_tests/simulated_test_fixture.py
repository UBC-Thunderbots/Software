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

from software.logger.logger import createLogger
from software.simulated_tests.pytest_main import load_command_line_arguments
from software.simulated_tests.tbots_test_runner import TbotsTestRunner

logger = createLogger(__name__)

LAUNCH_DELAY_S = 0.1
WORLD_BUFFER_TIMEOUT = 0.5
PROCESS_BUFFER_DELAY_S = 0.01
PAUSE_AFTER_FAIL_DELAY_S = 3


class SimulatorTestRunner(TbotsTestRunner):
    """Run a simulated test"""

    def __init__(
        self,
        test_name,
        thunderscope,
        simulator_proto_unix_io,
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

        super(SimulatorTestRunner, self).__init__(
            test_name,
            thunderscope,
            blue_full_system_proto_unix_io,
            yellow_full_system_proto_unix_io,
            gamecontroller,
        )
        self.simulator_proto_unix_io = simulator_proto_unix_io

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

    def set_worldState(self, worldstate: WorldState):
        self.simulator_proto_unix_io.send_proto(WorldState, worldstate)

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

            time_elapsed_s = 0

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

                # log data
                for logger_obj in data_loggers:
                    logger_obj.log_data(world, time_elapsed_s)

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

            self._stop_tscope()

        self._run_with_tscope(__runner)


def simulated_test_initializer():
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
        should_restart_on_crash=False,
    ) as blue_fs, FullSystem(
        f"{args.yellow_full_system_runtime_dir}/test/{test_name}",
        args.debug_yellow_full_system,
        True,
        should_restart_on_crash=False,
    ) as yellow_fs:
        with Gamecontroller(
            supress_logs=(not args.show_gamecontroller_logs), ci_mode=True,
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

            runner = SimulatorTestRunner(
                current_test,
                tscope,
                simulator_proto_unix_io,
                blue_full_system_proto_unix_io,
                yellow_full_system_proto_unix_io,
                gamecontroller,
            )

            # Only validate on the blue worlds
            blue_full_system_proto_unix_io.register_observer(World, runner.world_buffer)

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

                yield runner
                print(
                    f"\n\n To replay this test for the blue team, go to the `src` folder and run \n./tbots.py run thunderscope --blue_log {blue_logger.log_folder}"
                )
                print(
                    f"\n\n To replay this test for the yellow team, go to the `src` folder and run \n./tbots.py run thunderscope --yellow_log {yellow_logger.log_folder}"
                )


@pytest.fixture
def simulated_test_runner():

    initializer = simulated_test_initializer()

    yield next(initializer)

    # teardown
    try:
        next(initializer)
    except StopIteration:
        pass
