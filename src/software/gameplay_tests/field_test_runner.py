import queue
import time
from typing import override

import software.python_bindings as tbots_cpp

from proto.import_all_protos import (
    AutoChipOrKick,
    BallCollisionType,
    DribblerMode,
    MaxAllowedSpeedMode,
    MoveTactic,
    ObstacleAvoidanceMode,
    ValidationProtoSet,
    World,
    WorldState,
)
from software.gameplay_tests.tbots_test_runner import TbotsTestRunner
from software.gameplay_tests.validation import validation
from software.logger.logger import create_logger

logger = create_logger(__name__)

WORLD_BUFFER_TIMEOUT = 5.0
LAUNCH_DELAY_S = 0.1

SETUP_POSITION_TOLERANCE_M = 0.1
SETUP_ORIENTATION_TOLERANCE_DEG = 15
SETUP_TIMEOUT_S = 20.0


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
        owns_thunderscope=True,
    ):
        """Initialize the FieldTestRunner

        :param test_name: The name of the test to run
        :param thunderscope: The Thunderscope to use, None if not used
        :param blue_full_system_proto_unix_io: The blue full system proto unix io to use
        :param yellow_full_system_proto_unix_io: The yellow full system proto unix io to use
        :param gamecontroller: The gamecontroller context managed instance
        :param robot_communication: The robot communication instance
        :param: is_yellow_friendly: if yellow is the friendly team
        :param owns_thunderscope: Whether this runner controls the Thunderscope
            lifecycle (False when binding to an open Thunderscope in test mode)
        """
        super(FieldTestRunner, self).__init__(
            test_name,
            thunderscope,
            blue_full_system_proto_unix_io,
            yellow_full_system_proto_unix_io,
            gamecontroller,
            is_yellow_friendly,
            owns_thunderscope=owns_thunderscope,
        )
        self.robot_communication = robot_communication

        self._survey_field_robots()

    @override
    def set_world_state(self, world_state: WorldState):
        """Drives the friendly robots to the positions in the given world state.

        :param world_state: The WorldState proto with the desired robot states
        """
        friendly_move_tactics = self._create_move_tactics(
            world_state.yellow_robots
            if self.is_yellow_friendly
            else world_state.blue_robots
        )

        if self.is_yellow_friendly:
            self.set_tactics(blue_tactics=None, yellow_tactics=friendly_move_tactics)
        else:
            self.set_tactics(blue_tactics=friendly_move_tactics, yellow_tactics=None)

        self._wait_for_robots_at_world_state(world_state)

    @override
    def set_tactics(self, blue_tactics={}, yellow_tactics={}):
        """Override AI tactics, remapping the friendly team's simulated robot ids
        to the robot ids that are actually available on the field.
        """
        if self.is_yellow_friendly:
            yellow_tactics = self._map_tactics_to_field_ids(yellow_tactics)
        else:
            blue_tactics = self._map_tactics_to_field_ids(blue_tactics)

        super().set_tactics(blue_tactics=blue_tactics, yellow_tactics=yellow_tactics)

    @override
    def _pre_run_setup(self, setup: (lambda: None)):
        """Wait for estop to be in play state before running setup

        :param setup: Function that sets up the world state
        """
        self._wait_for_estop_play()
        if self._is_cancelled():
            return
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

        while time_elapsed_s < test_timeout_s and not self._is_cancelled():
            processing_start_time = time.time()

            # Check for new GC commands at this time step
            for delay, cmd, team in gc_cmd_with_delay:
                # If delay matches time
                if delay <= time_elapsed_s:
                    # send command
                    self.gamecontroller.send_gc_command(gc_command=cmd, team=team)
                    # remove command from the list
                    gc_cmd_with_delay.remove((delay, cmd, team))

            # Try to get the world; if can't receive for timeout,
            # something went wrong with either FullSystem or SSL Vision
            try:
                world = self.world_buffer.get(block=True, timeout=WORLD_BUFFER_TIMEOUT)
            except queue.Empty:
                raise Exception(
                    f"No World was received for {WORLD_BUFFER_TIMEOUT} seconds. Ending test early."
                )

            # The world reports robots by their field id, but validations are
            # written against simulated ids. Relabel so validations match.
            world = self._relabel_world_to_sim_ids(world)

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

            time_elapsed_s += time.time() - processing_start_time

        # Check that all eventually validations are eventually valid
        validation.check_validation(eventually_validation_proto_set)

        self._stopper()

    def _wait_for_estop_play(self):
        """Blocks until the estop is in the PLAY state, or the run is cancelled."""
        if self.robot_communication.estop_is_playing:
            return

        logger.info("\x1b[33m" + "Waiting for Estop to be in PLAY state..." + "\x1b[0m")
        while (
            not self.robot_communication.estop_is_playing and not self._is_cancelled()
        ):
            time.sleep(0.1)

        if self._is_cancelled():
            return

        logger.info(
            "\x1b[32m" + "Estop is in PLAY state. Proceeding with test." + "\x1b[0m"
        )

    def _survey_field_robots(self):
        """Surveys robots on the field and creates mappings to simulated ids.

        Simulated tests create robots with contiguous ids (0, 1, 2, ...), but
        field robots may have arbitrary ids. Maps each simulated id to an available
        field id, in ascending order, so set tactics and validations are sent to
        available field robots. e.g. field robots [2, 5] map to simuated [0, 1].
        """
        survey_start_time = time.time()
        self.friendly_robot_ids_field = []
        while time.time() - survey_start_time < WORLD_BUFFER_TIMEOUT:
            try:
                world = self.world_buffer.get(
                    block=True, timeout=0.1, return_cached=False
                )
                self.friendly_robot_ids_field = [
                    robot.id for robot in world.friendly_team.team_robots
                ]

                if len(self.friendly_robot_ids_field) > 0:
                    logger.info(f"Friendly team ids {self.friendly_robot_ids_field}")
                    break
            except queue.Empty:
                continue

        self.sim_to_field_robot_id = {
            sim_id: field_id
            for sim_id, field_id in enumerate(sorted(self.friendly_robot_ids_field))
        }
        self.field_to_sim_robot_id = {
            field_id: sim_id for sim_id, field_id in self.sim_to_field_robot_id.items()
        }
        logger.info(f"Simulated id to field id mapping {self.sim_to_field_robot_id}")

    def _create_move_tactics(self, robot_states):
        """Create a MoveTactic for each robot to drive it to its world state.

        :param robot_states: map of robot_id -> RobotState
        :return: dict of robot_id -> MoveTactic
        """
        return {
            robot_id: MoveTactic(
                destination=robot_state.global_position,
                final_orientation=robot_state.global_orientation,
                dribbler_mode=DribblerMode.OFF,
                ball_collision_type=BallCollisionType.AVOID,
                auto_chip_or_kick=AutoChipOrKick(),
                max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
                obstacle_avoidance_mode=ObstacleAvoidanceMode.SAFE,
            )
            for robot_id, robot_state in robot_states.items()
        }

    def _map_tactics_to_field_ids(self, tactics):
        """Remap a tactics dict keyed by simulated robot ids to field ids.

        :param tactics: None or dict of simulated_robot_id -> tactic
        :return: None if tactics is None, else dict of field_robot_id -> tactic
        """
        if tactics is None:
            return None

        mapped_tactics = {}
        for sim_id, tactic in tactics.items():
            if sim_id in self.sim_to_field_robot_id:
                mapped_tactics[self.sim_to_field_robot_id[sim_id]] = tactic

        return mapped_tactics

    def _relabel_world_to_sim_ids(self, world: World) -> World:
        """Return a copy of the world with friendly robot ids translated from
        field ids back to the simulated ids the test validations use.

        :param world: The World proto reported by the field full system
        :return: a copy of the world with friendly robot ids in simulated-id space
        """
        relabeled_world = World()
        relabeled_world.CopyFrom(world)
        for robot in relabeled_world.friendly_team.team_robots:
            if robot.id in self.field_to_sim_robot_id:
                robot.id = self.field_to_sim_robot_id[robot.id]
        return relabeled_world

    def _wait_for_robots_at_world_state(self, world_state: WorldState):
        """Block until robots in the world are close enough to its target position and orientation.

        :param world_state: The WorldState proto with the desired robot states
        :raises Exception: if the robots do not reach their targets within SETUP_TIMEOUT_S
        """
        friendly_targets = (
            world_state.yellow_robots
            if self.is_yellow_friendly
            else world_state.blue_robots
        )

        # Remap the friendly targets onto the field ids they were commanded to,
        # dropping any simulated id that has no available field robot
        friendly_targets = {
            self.sim_to_field_robot_id[sim_id]: robot_state
            for sim_id, robot_state in friendly_targets.items()
            if sim_id in self.sim_to_field_robot_id
        }

        logger.info("Waiting for robots to reach their target positions...")
        wait_start_time = time.time()
        while time.time() - wait_start_time < SETUP_TIMEOUT_S:
            if self._is_cancelled():
                return

            try:
                world = self.world_buffer.get(
                    block=True, timeout=0.1, return_cached=False
                )
            except queue.Empty:
                continue

            friendly_states = {
                robot.id: robot.current_state
                for robot in world.friendly_team.team_robots
            }
            if self._robots_all_at_target(friendly_targets, friendly_states):
                logger.info("All robots reached their target positions")
                return

        raise Exception(
            f"robots did not reach their target positions within {SETUP_TIMEOUT_S} seconds"
        )

    @staticmethod
    def _robots_all_at_target(targets, states):
        """Checks if all robots have reached their target positions and orientations
        within a certain threshold.

        :param targets: map of field_robot_id -> RobotState with the desired state
        :param states: dict of field_robot_id -> current RobotState
        :return: True if all friendly robots have reached their target state.
        """
        for robot_id, target in targets.items():
            if robot_id not in states:
                return False

            current = states[robot_id]
            distance = (
                tbots_cpp.createPoint(current.global_position)
                - tbots_cpp.createPoint(target.global_position)
            ).length()

            orientation_error = (
                tbots_cpp.createAngle(target.global_orientation)
                .minDiff(tbots_cpp.createAngle(current.global_orientation))
                .toDegrees()
            )

            if (
                distance > SETUP_POSITION_TOLERANCE_M
                or orientation_error > SETUP_ORIENTATION_TOLERANCE_DEG
            ):
                return False

        return True
