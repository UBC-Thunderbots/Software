import queue
import threading
import time
from typing import override

import pytest
from software.py_constants import MILLISECONDS_PER_SECOND

from proto.import_all_protos import *
from software.gameplay_tests.tbots_test_runner import TbotsTestRunner
from software.gameplay_tests.validation import validation
from software.logger.logger import create_logger
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

logger = create_logger(__name__)

LAUNCH_DELAY_S = 0.1
WORLD_BUFFER_TIMEOUT = 0.5
TICK_DURATION_S = 1 / 60


class SimulatedTestRunner(TbotsTestRunner):
    """Run a simulated test"""

    def __init__(
        self,
        test_name,
        thunderscope,
        simulator_proto_unix_io,
        blue_full_system_proto_unix_io,
        yellow_full_system_proto_unix_io,
        gamecontroller,
        ci_mode=False,
    ):
        """Initialize the SimulatorTestRunner

        :param test_name: The name of the test to run
        :param thunderscope: The Thunderscope to use, None if not used
        :param simulator_proto_unix_io: The simulator proto unix io to use
        :param blue_full_system_proto_unix_io: The blue full system proto unix io to use
        :param yellow_full_system_proto_unix_io: The yellow full system proto unix io to use
        :param gamecontroller: The gamecontroller context managed instance
        :param ci_mode: Run test as fast as possible
        """
        super(SimulatedTestRunner, self).__init__(
            test_name,
            thunderscope,
            blue_full_system_proto_unix_io,
            yellow_full_system_proto_unix_io,
            gamecontroller,
        )
        self.simulator_proto_unix_io = simulator_proto_unix_io
        self.ci_mode = ci_mode

    @override
    def set_world_state(self, worldstate: WorldState):
        """Sets the simulation worldstate

        :param worldstate: proto containing the desired worldstate
        """
        self.simulator_proto_unix_io.send_proto(WorldState, worldstate)

    @override
    def run_test(
        self,
        setup=lambda: None,
        always_validation_sequence_set=[[]],
        eventually_validation_sequence_set=[[]],
        test_timeout_s=3,
        gc_cmd_with_delay=[],
    ):
        """Helper function to run a test, with thunderscope if enabled

        :param always_validation_sequence_set: validation that should always be true
        :param eventually_validation_sequence_set: validation that should eventually be true
        :param setup: function that sets up the World state and the gamecontroller before running the test
        :param test_timeout_s: how long the test should run before timing out
        :param gc_cmd_with_delay: A list consisting of tuples with a duration and GC command, e.g.
                                  [
                                      (time, command, team),
                                      (time, command, team),
                                      ...
                                  ]
        """
        threading.excepthook = self.excepthook

        self._sync_setup(setup)

        # If thunderscope is enabled, run the test in a thread and show
        # thunderscope on this thread. The excepthook is setup to catch
        # any test failures and propagate them to the main thread
        if self.thunderscope:
            run_sim_thread = threading.Thread(
                target=self._runner,
                daemon=True,
                args=[
                    always_validation_sequence_set,
                    eventually_validation_sequence_set,
                    test_timeout_s,
                    gc_cmd_with_delay,
                ],
            )
            run_sim_thread.start()
            self.thunderscope.show()
            run_sim_thread.join()

            if self.last_exception:
                pytest.fail(str(self.last_exception))

        # If thunderscope is disabled, just run the test
        else:
            self._runner(
                always_validation_sequence_set,
                eventually_validation_sequence_set,
                test_timeout_s,
                gc_cmd_with_delay=gc_cmd_with_delay,
            )

    def _sync_setup(self, setup):
        """Run setup until simulator has received game state

        :param setup: Function that sets up the world state
        :param param: Parameter passed into setup
        """
        world_state_received_buffer = ThreadSafeBuffer(1, WorldStateReceivedTrigger)
        self.simulator_proto_unix_io.register_observer(
            WorldStateReceivedTrigger, world_state_received_buffer
        )

        while True:
            setup()

            try:
                world_state_received_buffer.get(
                    block=True, timeout=WORLD_BUFFER_TIMEOUT
                )
            except queue.Empty:
                # Did not receive a response within timeout period
                continue
            else:
                # Received a response from the simulator
                break

    def _runner(
        self,
        always_validation_sequence_set,
        eventually_validation_sequence_set,
        test_timeout_s,
        gc_cmd_with_delay,
    ):
        """Run a test

        :param always_validation_sequence_set: Validation functions that should
                                hold on every tick
        :param eventually_validation_sequence_set: Validation that should
                                eventually be true, before the test ends
        :param test_timeout_s: The timeout for the test, if any eventually_validations
                                remain after the timeout, the test fails.
        :param gc_cmd_with_delay: A list consisting of tuples with a duration and GC command, e.g.
                                  [
                                      (time, command, team),
                                      (time, command, team),
                                      ...
                                  ]
        """
        time_elapsed_s = 0

        while time_elapsed_s < test_timeout_s:
            # get time before we execute the loop
            processing_start_time = time.time()

            # Check for new GC commands at this time step
            for delay, cmd, team in gc_cmd_with_delay:
                # If delay matches time
                if delay <= time_elapsed_s:
                    # send command
                    self.gamecontroller.send_gc_command(gc_command=cmd, team=team)
                    # remove command from the list
                    gc_cmd_with_delay.remove((delay, cmd, team))

            tick = SimulatorTick(milliseconds=TICK_DURATION_S * MILLISECONDS_PER_SECOND)
            self.simulator_proto_unix_io.send_proto(SimulatorTick, tick)
            time_elapsed_s += TICK_DURATION_S

            while True:
                try:
                    world = self.world_buffer.get(
                        block=True, timeout=WORLD_BUFFER_TIMEOUT, return_cached=False
                    )

                    # We block until the timeout for the new primitives from AI. if not found still,
                    # the SSL Wrapper packet is resent in a loop until we actually get a primitive set from AI
                    # Otherwise, if the AI misses the first SSL Wrapper packet and doesn't start
                    # the simulated test will continue to tick forward, causes syncing issues with the AI
                    self.primitive_set_buffer.get(
                        block=True, timeout=WORLD_BUFFER_TIMEOUT, return_cached=False
                    )

                    break
                except queue.Empty:
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

            # get the time difference after we get the primitive (after any blocking that happened)
            processing_time = time.time() - processing_start_time

            # if the time we have blocked is less than a tick, sleep for the remaining time (for Thunderscope only)
            if (
                self.thunderscope
                and not self.ci_mode
                and TICK_DURATION_S > processing_time
            ):
                time.sleep(TICK_DURATION_S - processing_time)

            # Validate
            (
                eventually_validation_proto_set,
                always_validation_proto_set,
            ) = validation.run_validation_sequence_sets(
                world,
                eventually_validation_sequence_set,
                always_validation_sequence_set,
            )

            # Set the test name
            eventually_validation_proto_set.test_name = self.test_name
            always_validation_proto_set.test_name = self.test_name

            # Send out the validation proto to the full system
            # for visualization and logging for replays.
            if self.is_yellow_friendly:
                self.yellow_full_system_proto_unix_io.send_proto(
                    ValidationProtoSet, eventually_validation_proto_set
                )
                self.yellow_full_system_proto_unix_io.send_proto(
                    ValidationProtoSet, always_validation_proto_set
                )
            else:
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

        self.stopper()
