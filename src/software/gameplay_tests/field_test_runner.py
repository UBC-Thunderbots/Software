import queue
import threading
import time
from typing import override

import pytest

from proto.import_all_protos import ValidationProtoSet, WorldState
from software.gameplay_tests.tbots_test_runner import TbotsTestRunner
from software.gameplay_tests.validation import validation
from software.logger.logger import create_logger

logger = create_logger(__name__)

WORLD_BUFFER_TIMEOUT = 5.0
PROCESS_BUFFER_DELAY_S = 0.01
PAUSE_AFTER_FAIL_DELAY_S = 3
LAUNCH_DELAY_S = 0.1


class FieldTestRunner(TbotsTestRunner):
    """Run a field test"""

    def __init__(
        self,
        test_name,
        thunderscope,
        blue_full_system_proto_unix_io,
        yellow_full_system_proto_unix_io,
        gamecontroller,
        robot_communication,
        publish_validation_protos=True,
        is_yellow_friendly=False,
    ):
        """Initialize the FieldTestRunner

        :param test_name: The name of the test to run
        :param thunderscope: The Thunderscope to use, None if not used
        :param blue_full_system_proto_unix_io: The blue full system proto unix io to use
        :param yellow_full_system_proto_unix_io: The yellow full system proto unix io to use
        :param gamecontroller: The gamecontroller context managed instance
        :param robot_communication: The robot communication instance
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
        self.robot_communication = robot_communication

        # self._survey_field_robots()

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
        gc_cmd_with_delay=[],
    ):
        """Run a test. In a field test this means beginning validation.

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

        threading.excepthook = self._excepthook

        run_test_thread = threading.Thread(
            target=self._runner,
            daemon=True,
            args=[
                always_validation_sequence_set,
                eventually_validation_sequence_set,
                test_timeout_s,
                gc_cmd_with_delay,
            ],
        )

        run_test_thread.start()

        self.thunderscope.show()

        run_test_thread.join()

        if self.last_exception:
            pytest.fail(str(self.last_exception))

    def _wait_for_estop_play(self):
        """Blocks until the estop is in the PLAY state"""
        if self.robot_communication.estop_is_playing:
            return

        logger.info("\x1b[33m" + "Waiting for Estop to be in PLAY state..." + "\x1b[0m")
        while not self.robot_communication.estop_is_playing:
            # We must process events if Thunderscope is running to keep it responsive
            # if self.thunderscope:
            #     from pyqtgraph.Qt import QtWidgets
            #
            #     QtWidgets.QApplication.processEvents()
            time.sleep(0.1)
        logger.info(
            "\x1b[32m" + "Estop is in PLAY state. Proceeding with test." + "\x1b[0m"
        )

    def _stopper(self, delay=PROCESS_BUFFER_DELAY_S):
        time.sleep(delay)

        if self.thunderscope:
            self.thunderscope.close()

    def _runner(
        self,
        always_validation_sequence_set,
        eventually_validation_sequence_set,
        test_timeout_s,
        gc_cmd_with_delay,  # TODO (#3744): implement this
    ):
        time.sleep(LAUNCH_DELAY_S)

        self._wait_for_estop_play()

        test_end_time = time.time() + test_timeout_s

        # Keep track if we started with any eventually validations
        has_eventually_validations = any(
            len(seq) > 0 for seq in eventually_validation_sequence_set
        )

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

            # Break if eventually validation passes
            if has_eventually_validations and all(
                len(seq) == 0 for seq in eventually_validation_sequence_set
            ):
                break

        validation.check_validation(eventually_validation_proto_set)

        self._stopper()

    def _excepthook(self, args):
        """This function is _critical_ for show_thunderscope to work.
        If the test Thread will raises an exception we won't be able to close
        the window from the main thread.

        :param args: The args passed in from the hook
        """
        self._stopper(delay=PAUSE_AFTER_FAIL_DELAY_S)
        self.last_exception = args.exc_value
        raise self.last_exception

    def _survey_field_robots(self):
        logger.info("determining robots on field")
        # survey field for available robot ids
        survey_start_time = time.time()
        self.friendly_robot_ids_field = []
        while time.time() - survey_start_time < WORLD_BUFFER_TIMEOUT:
            try:
                world = self.world_buffer.get(block=True, timeout=0.1)
                self.initial_world = world
                self.friendly_robot_ids_field = [
                    robot.id for robot in world.friendly_team.team_robots
                ]

                if len(self.friendly_robot_ids_field) > 0:
                    logger.info(f"friendly team ids {self.friendly_robot_ids_field}")
                    break
            except queue.Empty:
                continue

        if len(self.friendly_robot_ids_field) == 0:
            raise Exception("no friendly robots found on field within timeout")
