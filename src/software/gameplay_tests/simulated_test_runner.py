import queue
import time
from typing import override

import proto.import_all_protos as protos
from software.gameplay_tests.tbots_test_runner import TbotsTestRunner
from software.gameplay_tests.validation import validation
from software.logger.logger import create_logger
from software.py_constants import (
    DEFAULT_SIMULATOR_TICK_RATE_SECONDS_PER_TICK,
    MILLISECONDS_PER_SECOND,
)
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

logger = create_logger(__name__)

LAUNCH_DELAY_S = 0.1
WORLD_BUFFER_TIMEOUT = 0.5
TICK_DURATION_S = DEFAULT_SIMULATOR_TICK_RATE_SECONDS_PER_TICK


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
        self.avah_ack_buffer = ThreadSafeBuffer(
            buffer_size=10, protobuf_type=protos.AvahAck
        )
        self.simulator_proto_unix_io.register_observer(
            protos.AvahAck, self.avah_ack_buffer
        )

    @override
    def set_world_state(self, worldstate: protos.WorldState):
        """Sets the simulation worldstate

        :param worldstate: proto containing the desired worldstate
        """
        self.simulator_proto_unix_io.send_proto(protos.WorldState, worldstate)

    @override
    def _pre_run_setup(self, setup: (lambda: None)):
        """Run setup until simulator has received game state

        :param setup: Function that sets up the world state
        """
        world_state_received_buffer = ThreadSafeBuffer(
            1, protos.WorldStateReceivedTrigger
        )
        self.simulator_proto_unix_io.register_observer(
            protos.WorldStateReceivedTrigger, world_state_received_buffer
        )

        setup()
        time.sleep(1)
        # while True:
        #     print("sending world state")
        #
        #     try:
        #         world_state_received_buffer.get(
        #             block=True, timeout=WORLD_BUFFER_TIMEOUT
        #         )
        #     except queue.Empty:
        #         # Did not receive a response within timeout period
        #         print("resending ehh")
        #         raise Exception("ReSENT")
        #         continue
        #     else:
        #         # Received a response from the simulator
        #         break

    @override
    def _runner(
        self,
        always_validation_sequence_set,
        eventually_validation_sequence_set,
        test_timeout_s,
        gc_cmd_with_delay,
    ):
        time.sleep(LAUNCH_DELAY_S)

        time_elapsed_s = 0

        while time_elapsed_s < test_timeout_s:
            processing_start_time = time.time()

            # Check for new GC commands at this time step
            for delay, cmd, team, point in gc_cmd_with_delay:
                # If delay matches time
                if delay <= time_elapsed_s:
                    # send command
                    self.gamecontroller.send_gc_command(
                        gc_command=cmd, team=team, final_ball_placement_point=point
                    )
                    # remove command from the list
                    gc_cmd_with_delay.remove((delay, cmd, team, point))
                    time.sleep(0.2)

            tick = protos.SimulatorTick(
                milliseconds=TICK_DURATION_S * MILLISECONDS_PER_SECOND
            )
            self.simulator_proto_unix_io.send_proto(protos.SimulatorTick, tick)
            time_elapsed_s += TICK_DURATION_S

            while True:
                try:
                    self.avah_ack_buffer.get(
                        block=True, timeout=1.0, return_cached=False
                    )
                    world = self.world_buffer.get(
                        block=True, timeout=WORLD_BUFFER_TIMEOUT, return_cached=False
                    )

                    break
                except queue.Empty:
                    raise RuntimeError("AVAH ACK NOT RECEIVED")
                    # If we timeout, that means full_system missed the last
                    # wrapper and robot status, lets resend it.
                    # logger.warning("Fullsystem missed last wrapper, resending ...")

                    # ssl_wrapper = self.ssl_wrapper_buffer.get(block=False)
                    # robot_status = self.robot_status_buffer.get(block=False)
                    #
                    # self.blue_full_system_proto_unix_io.send_proto(
                    #     protos.SSL_WrapperPacket, ssl_wrapper
                    # )
                    # self.blue_full_system_proto_unix_io.send_proto(
                    #     protos.RobotStatus, robot_status
                    # )

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
                    protos.ValidationProtoSet, eventually_validation_proto_set
                )
                self.yellow_full_system_proto_unix_io.send_proto(
                    protos.ValidationProtoSet, always_validation_proto_set
                )
            else:
                self.blue_full_system_proto_unix_io.send_proto(
                    protos.ValidationProtoSet, eventually_validation_proto_set
                )
                self.blue_full_system_proto_unix_io.send_proto(
                    protos.ValidationProtoSet, always_validation_proto_set
                )

            if any(len(seq) > 0 for seq in always_validation_sequence_set):
                # Check that all always validations are always valid
                validation.check_validation(always_validation_proto_set)
            else:
                # If there are no always validations, check eventually validations to end test early
                try:
                    validation.check_validation(eventually_validation_proto_set)
                    break
                except AssertionError:
                    pass

        # Check that all eventually validations are eventually valid
        validation.check_validation(eventually_validation_proto_set)

        self._stopper()
