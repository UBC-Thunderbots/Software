import queue
import threading
import time
from typing import override

import pytest
from proto.ssl_gc_common_pb2 import Team
from proto.ssl_gc_state_pb2 import Command

from proto.import_all_protos import ValidationProtoSet, WorldState
from software.gameplay_tests.tbots_test_runner import TbotsTestRunner
from software.gameplay_tests.validation import validation
from software.logger.logger import create_logger

logger = create_logger(__name__)

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
        is_yellow_friendly=False,
    ):
        """Initialize the FieldTestRunner

        :param test_name: The name of the test to run
        :param thunderscope: The Thunderscope to use, None if not used
        :param blue_full_system_proto_unix_io: The blue full system proto unix io to use
        :param yellow_full_system_proto_unix_io: The yellow full system proto unix io to use
        :param gamecontroller: The gamecontroller context managed instance
        :param publish_validation_protos: whether to publish validation protos
        :param: is_yellow_friendly: if yellow is the friendly team
        """
        super(FieldTestRunner, self).__init__(
            test_name,
            thunderscope,
            blue_full_system_proto_unix_io,
            yellow_full_system_proto_unix_io,
            gamecontroller,
            is_yellow_friendly,
        )
        self.publish_validation_protos = publish_validation_protos
        self.is_yellow_friendly = is_yellow_friendly

        logger.info("determining robots on field")
        # survey field for available robot ids
        try:
            world = self.world_buffer.get(block=True, timeout=WORLD_BUFFER_TIMEOUT)
            self.initial_world = world
            self.friendly_robot_ids_field = [
                robot.id for robot in world.friendly_team.team_robots
            ]

            logger.info(f"friendly team ids {self.friendly_robot_ids_field}")

            if len(self.friendly_robot_ids_field) == 0:
                raise Exception("no friendly robots found on field")

        except queue.Empty:
            raise Exception(
                f"No Worlds were received with in {WORLD_BUFFER_TIMEOUT} seconds. \
                Please make sure atleast 1 robot and 1 ball is present on the field."
            )

    @override
    def send_gamecontroller_command(
        self,
        gc_command: Command,
        team: Team,
        final_ball_placement_point=None,
    ):
        """Send a command to the gamecontroller

        :param gc_command: The command to send
        :param team: The team which the command as attributed to
        :param final_ball_placement_point: The ball placement point
        """
        self.gamecontroller.send_ci_input(
            gc_command=gc_command,
            team=team,
            final_ball_placement_point=final_ball_placement_point,
        )

    @override
    def set_world_state(self, world_state: WorldState):
        logger.warning(
            "set_world_state called in field test: "
            "assuming robots are initialized according to the given parameters"
        )

    @override
    def run_test(
        self,
        setup=lambda: None,
        always_validation_sequence_set=[[]],
        eventually_validation_sequence_set=[[]],
        test_timeout_s=3,
        # Unused parameters, here for compatibility with sim tests
        tick_duration_s=None,
        ci_cmd_with_delay=None,
        run_till_end=None,
    ):
        """Run a test. In a field test this means beginning validation.

        :param always_validation_sequence_set: Validation functions that should
                                hold on every tick
        :param eventually_validation_sequence_set: Validation that should
                                eventually be true, before the test ends
        :param test_timeout_s: The timeout for the test, if any eventually_validations
                                remain after the timeout, the test fails.
        """

        def stop_test(delay):
            time.sleep(delay)
            if self.thunderscope:
                self.thunderscope.close()

        def __runner():
            time.sleep(LAUNCH_DELAY_S)

            test_end_time = time.time() + test_timeout_s

            while time.time() < test_end_time:
                while True:
                    try:
                        world = self.world_buffer.get(
                            block=True, timeout=WORLD_BUFFER_TIMEOUT
                        )
                        break
                    except queue.Empty:
                        # If we timeout, that means full_system missed the last
                        # wrapper and robot status, lets resend it.
                        logger.warning(
                            f"No World was received for {WORLD_BUFFER_TIMEOUT} seconds. Ending test early."
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
            stop_test(TEST_END_DELAY)

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
            run_test_thread = threading.Thread(target=__runner, daemon=True)
            run_test_thread.start()
            self.thunderscope.show()
            run_test_thread.join()

            if self.last_exception:
                pytest.fail(str(self.last_exception))

        else:
            __runner()
