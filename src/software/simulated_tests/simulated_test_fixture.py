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
from proto.message_translation import tbots_protobuf
from software.py_constants import *


logger = createLogger(__name__)

LAUNCH_DELAY_S = 0.1
WORLD_BUFFER_TIMEOUT = 0.5
PROCESS_BUFFER_DELAY_S = 0.01
PAUSE_AFTER_FAIL_DELAY_S = 3
DEFAULT_SIM_TICK_DURATION_S = 0.0166  # default to 60HZ


class SimulatorTestRunner(TbotsTestRunner):
    """Run a simulated test"""

    def __init__(
        self,
        test_name,
        simulator_proto_unix_io,
        blue_full_system_proto_unix_io,
        yellow_full_system_proto_unix_io,
        gamecontroller,
        publish_validation_protos=True,
        simulation_tick_duration_s=DEFAULT_SIM_TICK_DURATION_S,
        sleep_between_ticks=True,
    ):
        """Initialize the SimulatorTestRunner
        
        :param test_name: The name of the test to run
        :param simulator_proto_unix_io: The simulator proto unix io to use
        :param blue_full_system_proto_unix_io: The blue full system proto unix io to use
        :param yellow_full_system_proto_unix_io: The yellow full system proto unix io to use
        :param gamecontroller: The gamecontroller context managed instance 
        :param publish_validation_protos: whether to publish validation protos
        :param simulation_tick_duration_s: duration in seconds of a simulation tick
        :param sleep_between_ticks: whether the simulation should pause between ticks


        """

        super(SimulatorTestRunner, self).__init__(
            test_name,
            blue_full_system_proto_unix_io,
            yellow_full_system_proto_unix_io,
            gamecontroller,
        )
        self.simulator_proto_unix_io = simulator_proto_unix_io
        self.publish_validation_protos = publish_validation_protos
        self.tick_duration_s = simulation_tick_duration_s
        self.sleep_between_ticks = sleep_between_ticks

    def set_worldState(self, worldstate: WorldState):
        """Sets the simulation worldstate

        Args:
            worldstate (WorldState): proto containing the desired worldstate
        """
        self.simulator_proto_unix_io.send_proto(WorldState, worldstate)

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
        """Run a test

        :param always_validation_sequence_set: Validation functions that should
                                hold on every tick
        :param eventually_validation_sequence_set: Validation that should
                                eventually be true, before the test ends
        :param test_timeout_s: The timeout for the test, if any eventually_validations
                                remain after the timeout, the test fails.
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
                    milliseconds=self.tick_duration_s * MILLISECONDS_PER_SECOND
                )
                self.simulator_proto_unix_io.send_proto(SimulatorTick, tick)
                time_elapsed_s += self.tick_duration_s

                if self.sleep_between_ticks:
                    time.sleep(self.tick_duration_s)
                time.sleep(self.tick_duration_s)

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

        __runner()


def simulated_test_initializer(
    simulator_proto_unix_io,
    yellow_full_system_proto_unix_io,
    blue_full_system_proto_unix_io,
    sleep_between_ticks=True,
):
    """Initializes a test runner object to be used as part of a pytest fixture, and which will be given as input to tests

    Args:
        simulator_proto_unix_io (_type_): simulator unix socket
        yellow_full_system_proto_unix_io (_type_): yellow ai unix socket
        blue_full_system_proto_unix_io (_type_): blue ai unix socket
        sleep_between_ticks (bool, optional): whether the simulation should pause between ticks (for example to allow a gui to catchup). Defaults to True.

    Yields:
        SimulatorTestRunner: yields a test runner to the pytest fixture. Statements made after the yield will be run after the test.  
    """
    args = load_command_line_arguments()

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

            runner = SimulatorTestRunner(
                current_test,
                simulator_proto_unix_io,
                blue_full_system_proto_unix_io,
                yellow_full_system_proto_unix_io,
                gamecontroller,
                simulation_tick_duration_s=DEFAULT_SIM_TICK_DURATION_S,
                sleep_between_ticks=sleep_between_ticks,
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
