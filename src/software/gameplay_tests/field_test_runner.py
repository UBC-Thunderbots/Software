import queue
import time
from typing import override

from proto.import_all_protos import ValidationProtoSet, WorldState
from software.gameplay_tests.tbots_test_runner import TbotsTestRunner
from software.gameplay_tests.validation import validation
from software.logger.logger import create_logger

logger = create_logger(__name__)

WORLD_BUFFER_TIMEOUT = 5.0
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
        is_yellow_friendly=False,
    ):
        """Initialize the FieldTestRunner

        :param test_name: The name of the test to run
        :param thunderscope: The Thunderscope to use, None if not used
        :param blue_full_system_proto_unix_io: The blue full system proto unix io to use
        :param yellow_full_system_proto_unix_io: The yellow full system proto unix io to use
        :param gamecontroller: The gamecontroller context managed instance
        :param robot_communication: The robot communication instance
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
        self.is_yellow_friendly = is_yellow_friendly
        self.robot_communication = robot_communication

        self._survey_field_robots()

    @override
    def set_world_state(self, world_state: WorldState):
        # TODO (#3369): add visualization for setup instead of just logging warning
        logger.warning(
            "set_world_state called in field test: "
            "assuming robots are initialized according to the given parameters"
        )

    @override
    def _pre_run_setup(self, setup: (lambda: None)):
        """Wait for estop to be in play state before runing setup

        :param setup: Function that sets up the world state
        """
        self._wait_for_estop_play()
        setup()

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
            for delay, cmd, team in gc_cmd_with_delay:
                # If delay matches time
                if delay <= time_elapsed_s:
                    # send command
                    self.gamecontroller.send_gc_command(gc_command=cmd, team=team)
                    # remove command from the list
                    gc_cmd_with_delay.remove((delay, cmd, team))

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

            # Set the test name
            eventually_validation_proto_set.test_name = self.test_name
            always_validation_proto_set.test_name = self.test_name

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

            if len(always_validation_sequence_set) != 0:
                # Check that all always validations are always valid
                validation.check_validation(always_validation_proto_set)
            else:
                # If there are no always validations, check eventually validations to end test early
                try:
                    validation.check_validation(eventually_validation_proto_set)
                    break
                except AssertionError:
                    pass

            time_elapsed_s += time.time() - processing_start_time

        # Check that all eventually validations are eventually valid
        validation.check_validation(eventually_validation_proto_set)

        self._stopper()

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
