import math
import queue
import time
from typing import override

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

# How close (in metres) a robot must be to its target position before the test
# is allowed to start
ROBOT_SETUP_TOLERANCE_M = 0.1
# How long to wait for robots to drive to their target positions before giving up
ROBOT_SETUP_TIMEOUT_S = 30.0


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
        self.is_yellow_friendly = is_yellow_friendly
        self.robot_communication = robot_communication

        self._survey_field_robots()

    @override
    def set_world_state(self, world_state: WorldState):
        """Send move tactics to robots to the positions in the given world state.

        :param world_state: The WorldState proto with the desired robot states
        """
        self.set_tactics(
            blue_tactics=self._create_move_tactics(world_state.blue_robots),
            yellow_tactics=self._create_move_tactics(world_state.yellow_robots),
        )

        self._wait_for_robots_at_world_state(world_state)

    @override
    def set_tactics(self, blue_tactics={}, yellow_tactics={}):
        """Override AI tactics, remapping each team's simulated robot ids to the
        robot ids that are actually available on the field.
        """
        if self.is_yellow_friendly:
            blue_sim_to_field_id = self.enemy_sim_to_field_id
            yellow_sim_to_field_id = self.friendly_sim_to_field_id
        else:
            blue_sim_to_field_id = self.friendly_sim_to_field_id
            yellow_sim_to_field_id = self.enemy_sim_to_field_id

        blue_tactics = self._map_tactics_to_field_ids(
            blue_tactics, blue_sim_to_field_id, "blue"
        )
        yellow_tactics = self._map_tactics_to_field_ids(
            yellow_tactics, yellow_sim_to_field_id, "yellow"
        )

        super().set_tactics(blue_tactics=blue_tactics, yellow_tactics=yellow_tactics)

    @override
    def _pre_run_setup(self, setup: (lambda: None)):
        """Wait for estop to be in play state before running setup

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
        """Blocks until the estop is in the PLAY state"""
        if self.robot_communication.estop_is_playing:
            return

        logger.info("\x1b[33m" + "Waiting for Estop to be in PLAY state..." + "\x1b[0m")
        while not self.robot_communication.estop_is_playing:
            time.sleep(0.1)
        logger.info(
            "\x1b[32m" + "Estop is in PLAY state. Proceeding with test." + "\x1b[0m"
        )

    def _survey_field_robots(self):
        logger.info("determining robots on field")
        # survey field for available robot ids
        survey_start_time = time.time()
        self.friendly_robot_ids_field = []
        self.enemy_robot_ids_field = []
        while time.time() - survey_start_time < WORLD_BUFFER_TIMEOUT:
            try:
                world = self.world_buffer.get(block=True, timeout=0.1)
                self.initial_world = world
                self.friendly_robot_ids_field = [
                    robot.id for robot in world.friendly_team.team_robots
                ]
                self.enemy_robot_ids_field = [
                    robot.id for robot in world.enemy_team.team_robots
                ]

                if len(self.friendly_robot_ids_field) > 0:
                    logger.info(f"friendly team ids {self.friendly_robot_ids_field}")
                    logger.info(f"enemy team ids {self.enemy_robot_ids_field}")
                    break
            except queue.Empty:
                continue

        if len(self.friendly_robot_ids_field) == 0:
            raise Exception("no friendly robots found on field within timeout")

        # Simulated tests create robots with contiguous ids (0, 1, 2, ...), but
        # the robots physically on the field may have arbitrary ids and there may
        # be fewer of them. Map each simulated id to an available field id, in
        # ascending order and per team, so tactics and validations written against
        # simulated ids refer to the robots that are actually present. e.g. field
        # robots [4, 5] map simulated id 0 -> 4 and 1 -> 5; field robots [0, 1, 5]
        # map 0 -> 0, 1 -> 1 and 2 -> 5.
        self.friendly_sim_to_field_id = self._build_sim_to_field_mapping(
            self.friendly_robot_ids_field
        )
        self.enemy_sim_to_field_id = self._build_sim_to_field_mapping(
            self.enemy_robot_ids_field
        )
        # Inverse mappings, used to relabel the world's field ids back to the
        # simulated ids that validations are written against.
        self.friendly_field_to_sim_id = {
            field_id: sim_id
            for sim_id, field_id in self.friendly_sim_to_field_id.items()
        }
        self.enemy_field_to_sim_id = {
            field_id: sim_id for sim_id, field_id in self.enemy_sim_to_field_id.items()
        }
        logger.info(
            f"friendly simulated id to field id mapping {self.friendly_sim_to_field_id}"
        )
        logger.info(
            f"enemy simulated id to field id mapping {self.enemy_sim_to_field_id}"
        )

    @staticmethod
    def _build_sim_to_field_mapping(field_ids):
        """Map contiguous simulated robot ids (0, 1, 2, ...) onto the available
        field ids for a team, in ascending order.

        :param field_ids: the robot ids present on the field for a team
        :return: dict of simulated_robot_id -> field_robot_id
        """
        return {sim_id: field_id for sim_id, field_id in enumerate(sorted(field_ids))}

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
                auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0.0),
                max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
                obstacle_avoidance_mode=ObstacleAvoidanceMode.SAFE,
            )
            for robot_id, robot_state in robot_states.items()
        }

    def _map_tactics_to_field_ids(self, tactics, sim_to_field_id, team_name):
        """Remap a tactics dict keyed by simulated robot ids to the robot ids
        available on the field. Tactics for simulated ids with no available
        field robot are dropped with a warning.

        :param tactics: None or dict of simulated_robot_id -> tactic
        :param sim_to_field_id: the team's simulated-id -> field-id mapping
        :param team_name: the team colour, used in warnings
        :return: None if tactics is None, else dict of field_robot_id -> tactic
        """
        if tactics is None:
            return None

        mapped_tactics = {}
        for sim_id, tactic in tactics.items():
            if sim_id not in sim_to_field_id:
                logger.warning(
                    f"no {team_name} field robot available for simulated robot id "
                    f"{sim_id}; skipping its tactic"
                )
                continue
            mapped_tactics[sim_to_field_id[sim_id]] = tactic

        return mapped_tactics

    @staticmethod
    def _remap_robot_state_ids(robot_states, sim_to_field_id):
        """Remap a world-state robot map keyed by simulated ids to field ids,
        dropping any simulated id with no available field robot.

        :param robot_states: map of simulated_robot_id -> RobotState
        :param sim_to_field_id: the team's simulated-id -> field-id mapping
        :return: dict of field_robot_id -> RobotState
        """
        return {
            sim_to_field_id[sim_id]: robot_state
            for sim_id, robot_state in robot_states.items()
            if sim_id in sim_to_field_id
        }

    def _relabel_world_to_sim_ids(self, world: World) -> World:
        """Return a copy of the world with both teams' robot ids translated from
        field ids back to the simulated ids the test uses.

        Validations match robots by the contiguous simulated ids (0, 1, 2, ...),
        but the world reports the robots' actual field ids. Relabeling the world
        here lets every id-based validation find the correct robot without each
        validation needing to know about the mapping. Robots with no mapping
        (e.g. extras not part of the test) keep their field id.

        :param world: The World proto reported by the field full system
        :return: a copy of the world with robot ids in simulated-id space
        """
        relabeled_world = World()
        relabeled_world.CopyFrom(world)
        self._relabel_team_to_sim_ids(
            relabeled_world.friendly_team, self.friendly_field_to_sim_id
        )
        self._relabel_team_to_sim_ids(
            relabeled_world.enemy_team, self.enemy_field_to_sim_id
        )
        return relabeled_world

    @staticmethod
    def _relabel_team_to_sim_ids(team, field_to_sim_id):
        """Relabel a team's robot ids in place from field ids to simulated ids.

        :param team: a Team proto whose robots' ids to relabel
        :param field_to_sim_id: the team's field-id -> simulated-id mapping
        """
        for robot in team.team_robots:
            if robot.id in field_to_sim_id:
                robot.id = field_to_sim_id[robot.id]

    def _wait_for_robots_at_world_state(self, world_state: WorldState):
        """Block until every robot in the world state is within
        ROBOT_SETUP_TOLERANCE_M of its target position.

        Robot positions are read from the friendly full system's World in the
        same fashion as _survey_field_robots. The friendly team maps to the
        friendly-coloured robots in the world state, and the enemy team to the
        other colour. Each team's targets are remapped from simulated ids to the
        field ids they were assigned to in set_tactics.

        :param world_state: The WorldState proto with the desired robot states
        :raises Exception: if the robots do not reach their targets within
                           ROBOT_SETUP_TIMEOUT_S
        """
        if self.is_yellow_friendly:
            friendly_targets = world_state.yellow_robots
            enemy_targets = world_state.blue_robots
        else:
            friendly_targets = world_state.blue_robots
            enemy_targets = world_state.yellow_robots

        # Remap each team's targets onto the field ids they were commanded to,
        # dropping any simulated id that has no available field robot
        friendly_targets = self._remap_robot_state_ids(
            friendly_targets, self.friendly_sim_to_field_id
        )
        enemy_targets = self._remap_robot_state_ids(
            enemy_targets, self.enemy_sim_to_field_id
        )

        logger.info("waiting for robots to reach their target positions")
        wait_start_time = time.time()
        while time.time() - wait_start_time < ROBOT_SETUP_TIMEOUT_S:
            try:
                world = self.world_buffer.get(block=True, timeout=0.1)
            except queue.Empty:
                continue

            friendly_positions = {
                robot.id: robot.current_state.global_position
                for robot in world.friendly_team.team_robots
            }
            enemy_positions = {
                robot.id: robot.current_state.global_position
                for robot in world.enemy_team.team_robots
            }

            if self._all_robots_at_targets(
                friendly_targets, friendly_positions
            ) and self._all_robots_at_targets(enemy_targets, enemy_positions):
                logger.info("all robots reached their target positions")
                return

        raise Exception(
            f"robots did not reach their target positions within "
            f"{ROBOT_SETUP_TIMEOUT_S} seconds"
        )

    @staticmethod
    def _all_robots_at_targets(targets, positions):
        """Check that every targeted robot is within ROBOT_SETUP_TOLERANCE_M of
        its target position.

        :param targets: map of robot_id -> RobotState with the desired positions
        :param positions: dict of robot_id -> current global_position Point
        :return: True if every targeted robot is present and within tolerance
        """
        for robot_id, robot_state in targets.items():
            if robot_id not in positions:
                return False

            target = robot_state.global_position
            current = positions[robot_id]
            distance = math.hypot(
                current.x_meters - target.x_meters,
                current.y_meters - target.y_meters,
            )
            if distance > ROBOT_SETUP_TOLERANCE_M:
                return False

        return True
