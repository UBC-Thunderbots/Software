import os
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.gl.layers.gl_layer import GLLayer
from dataclasses import dataclass
import software.python_bindings as tbots_cpp
from software.thunderscope.constants import RuntimeManagerConstants
import logging
from proto.visualization_pb2 import AttackerVisualization
from proto.import_all_protos import *
from software.py_constants import ROBOT_MAX_RADIUS_METERS
from typing import override


@dataclass
class FSStats:
    """Stats for how well a FullSystem is performing"""

    num_yellow_cards: int = 0
    num_red_cards: int = 0
    num_scores: int = 0

    num_shots_on_net: int = 0
    latest_shot_angle: tbots_cpp.Angle = tbots_cpp.Angle()
    shot_taken: bool = False

    is_shot_incoming: bool = False
    num_enemy_shots_blocked: int = 0


class GlFSStatsLayer(GLLayer):
    # From GoalieTacticConfig
    INCOMING_SHOT_MIN_VELOCITY = 0.2

    # tune these values to reduce noise in what is considered a "shot" to net
    # higher values exclude noise such as dribbling or passes
    # but can exclude real kicks
    MIN_SHOT_SPEED = 2.0
    MAX_KICK_ANGLE_DIFFERENCE = tbots_cpp.Angle.fromDegrees(5)
    MIN_NEW_SHOT_ANGLE_DIFFERENCE_RAD = tbots_cpp.Angle.fromDegrees(10).toRadians()

    def __init__(
        self,
        name: str,
        friendly_colour_yellow: bool,
        buffer_size: int = 5,
        record_enemy_stats: bool = False,
    ):
        """Initializes a GlFSStatsLayer

        :param name: the display name of this layer
        :param friendly_colour_yellow: if the friendly colour is yellow
        :param buffer_size: the buffer size for protocol buffers
        :param record_enemy_stats: if this layer should record both friendly and enemy stats or just friendly
        """
        super().__init__(name)

        self.friendly_colour_yellow = friendly_colour_yellow

        # True if enemy had the last possession, False if friendly
        # None if neither
        self.last_possession_enemy: bool | None = None

        self.attacker_vis_buffer = ThreadSafeBuffer(buffer_size, AttackerVisualization)
        self.referee_buffer = ThreadSafeBuffer(buffer_size, Referee)
        self.world_buffer = ThreadSafeBuffer(buffer_size, World)

        self.stats = FSStats()

        self.record_enemy_stats = record_enemy_stats
        if self.record_enemy_stats:
            self.enemy_stats = FSStats()

    @override
    def refresh_graphics(self) -> None:
        """Refreshes the stats for the game so far"""
        world_msg = self.world_buffer.get(block=False, return_cached=True)
        world = tbots_cpp.World(world_msg)

        self._update_posession(world)

        self._record_attacker_stats(world.ball())

        self._record_goalie_stats(world.ball(), world.field())

        self._record_referee_stats()

        self._flush_stats()

    def _update_posession(self, world: tbots_cpp.World) -> None:
        """Updates the team which last had the ball

        :param world: the current state of the world
        """
        friendly_team, enemy_team = (
            (world.enemyTeam(), world.friendlyTeam())
            if self.friendly_colour_yellow
            else (world.friendlyTeam(), world.enemyTeam())
        )

        self.last_possession_enemy = self._check_posession_for_teams(
            friendly_team, enemy_team, world.ball().position()
        )

    def _is_goal_shot_incoming(
        self, ball: tbots_cpp.Ball, field: tbots_cpp.Field, for_friendly: bool
    ) -> bool:
        """Follows similar logic to goalie_fsm.cpp
        Checks if the ball velocity intersects with the goal line given
        and if the ball is moving fast enough in the right direction
        and if the ball is in the correct half of the field
        and if the last possession was by the correct team
        to consider it a shot on goal

        :param ball: the current state of the ball
        :param field: the current state of the field
        :param for_friendly: if we should check for a goal shot on the friendly or enemy side
        :return:
        """
        ball_position = ball.position()
        ball_velocity = ball.velocity()
        ball_ray = tbots_cpp.Ray(ball_position, ball_velocity)
        goal_segment = (
            tbots_cpp.Segment(
                field.friendlyGoalpostPos()
                + tbots_cpp.Vector(0, -ROBOT_MAX_RADIUS_METERS),
                field.friendlyGoalpostNeg()
                + tbots_cpp.Vector(0, ROBOT_MAX_RADIUS_METERS),
            )
            if for_friendly
            else tbots_cpp.Segment(
                field.enemyGoalpostPos()
                + tbots_cpp.Vector(0, -ROBOT_MAX_RADIUS_METERS),
                field.enemyGoalpostNeg() + tbots_cpp.Vector(0, ROBOT_MAX_RADIUS_METERS),
            )
        )

        shot_incoming = (
            len(tbots_cpp.intersection(ball_ray, goal_segment)) != 0
            and ball_velocity.length() > self.MIN_SHOT_SPEED
            and (
                field.pointInFriendlyHalf(ball_position)
                if for_friendly
                else field.pointInEnemyHalf(ball_position)
            )
            and (ball_velocity.x() <= 0 if for_friendly else ball_velocity.x() >= 0)
        )

        return shot_incoming

    def _record_goalie_stats(
        self, ball: tbots_cpp.Ball, field: tbots_cpp.Field
    ) -> None:
        """Record stats related to the goalie
        So shots taken and blocked on goal

        :param ball: the current state of the ball
        :param field: the current state of the field
        """
        friendly_shot_incoming = self._is_goal_shot_incoming(
            ball, field, for_friendly=True
        )

        # assume that if a ball was previously incoming and then was no longer incoming
        # then we successfully blocked it
        # TODO: verify if this is accurate
        if not friendly_shot_incoming and self.stats.is_shot_incoming:
            self.stats.num_enemy_shots_blocked += 1

        if self.record_enemy_stats:
            # if there wasn't an incoming shot and now there is, the enemy must have taken a shot at us
            if friendly_shot_incoming and not self.stats.is_shot_incoming:
                self.enemy_stats.num_shots_on_net += 1

            # same logic, but from the enemy's perspective
            enemy_shot_incoming = self._is_goal_shot_incoming(
                ball, field, for_friendly=False
            )

            if not enemy_shot_incoming and self.enemy_stats.is_shot_incoming:
                self.enemy_stats.num_enemy_shots_blocked += 1

            self.enemy_stats.is_shot_incoming = enemy_shot_incoming

        self.stats.is_shot_incoming = friendly_shot_incoming

    def _check_posession_for_teams(
        self,
        friendly_team: tbots_cpp.Team,
        enemy_team: tbots_cpp.Team,
        ball_position: tbots_cpp.Point,
    ) -> bool | None:
        """Check for which team has possession of the ball
        True if enemy team, False if friendly team, None if neither

        :param friendly_team: the friendly team
        :param enemy_team: the enemy team
        :param ball_position: the current ball position
        :return: True / False / None depending on which team has possession
        """
        if self._check_posession_for_team(enemy_team, ball_position):
            return True

        if self._check_posession_for_team(friendly_team, ball_position):
            return False

        return None

    def _check_posession_for_team(
        self, team: tbots_cpp.Team, ball_position: tbots_cpp.Point
    ) -> bool:
        """Check if the given team has possession of the ball

        :param team: the team to check
        :param ball_position: the current ball position
        :return: True if the team has possession, False otherwise
        """
        for robot in team.getAllRobots():
            if robot.isNearDribbler(ball_position):
                return True

        return False

    def _update_latest_shot_angle(self) -> None:
        """Update the latest shot angle if there is a new shot from the attacker
        that is different enough from the old shot
        """
        attacker_vis_msg = self.attacker_vis_buffer.get(block=False)

        if attacker_vis_msg and attacker_vis_msg.HasField("shot"):
            shot_origin = tbots_cpp.Point(
                attacker_vis_msg.shot.shot_origin.x_meters,
                attacker_vis_msg.shot.shot_origin.y_meters,
            )
            shot_target = tbots_cpp.Point(
                attacker_vis_msg.shot.shot_target.x_meters,
                attacker_vis_msg.shot.shot_target.y_meters,
            )
            new_shot_angle = tbots_cpp.Vector(
                shot_target.x() - shot_origin.x(), shot_target.y() - shot_origin.y()
            ).orientation()

            if (
                abs((new_shot_angle - self.stats.latest_shot_angle).toRadians())
                > self.MIN_NEW_SHOT_ANGLE_DIFFERENCE_RAD
            ):
                self.stats.latest_shot_angle = new_shot_angle
                self.stats.shot_taken = False

    def _record_attacker_stats(self, ball: tbots_cpp.Ball) -> None:
        """Record stats related to the attacker
        i.e the shots taken on goal by the friendly team
        Checks if the shot has actually been taken or not

        :param ball: the current state of the ball
        """
        self._update_latest_shot_angle()

        if not self.stats.shot_taken and ball.hasBallBeenKicked(
            self.stats.latest_shot_angle,
            self.MIN_SHOT_SPEED,
            self.MAX_KICK_ANGLE_DIFFERENCE,
        ):
            self.stats.num_shots_on_net += 1
            self.stats.shot_taken = True

    def _record_referee_stats(self) -> None:
        """Record stats related to the referee
        such as game score and red / yellow cards
        """
        refree_msg = self.referee_buffer.get(block=False, return_cached=True)

        if refree_msg.HasField("yellow" if self.friendly_colour_yellow else "blue"):
            self._record_referee_stats_per_team(
                refree_msg.yellow if self.friendly_colour_yellow else refree_msg.blue,
                self.stats,
            )

        if self.record_enemy_stats and refree_msg.HasField(
            "blue" if self.friendly_colour_yellow else "yellow"
        ):
            self._record_referee_stats_per_team(
                refree_msg.blue if self.friendly_colour_yellow else refree_msg.yellow,
                self.enemy_stats,
            )

    def _record_referee_stats_per_team(
        self, team_info: TeamInfo, stats: FSStats
    ) -> None:
        """Record stats related to the referee for the given team

        :param team_info: the information about the team
        :param stats: the stats to update
        """
        if team_info.HasField("score"):
            stats.num_scores = team_info.score

        if team_info.HasField("yellow_cards"):
            stats.num_yellow_cards = team_info.yellow_cards

        if team_info.HasField("red_cards"):
            stats.num_red_cards = team_info.red_cards

    def _flush_stats(self):
        """Write the current stats to disk"""
        stats_file_name = (
            RuntimeManagerConstants.RUNTIME_ENEMY_STATS_FILE
            if self.friendly_colour_yellow
            else RuntimeManagerConstants.RUNTIME_FRIENDLY_STATS_FILE
        )

        self._write_stats_to_file(self.stats, stats_file_name)

        if self.record_enemy_stats:
            enemy_stats_file_name = (
                RuntimeManagerConstants.RUNTIME_FRIENDLY_FROM_ENEMY_STATS_FILE
                if self.friendly_colour_yellow
                else RuntimeManagerConstants.RUNTIME_ENEMY_FROM_FRIENDLY_STATS_FILE
            )

            self._write_stats_to_file(self.enemy_stats, enemy_stats_file_name)

    def _write_stats_to_file(self, stats: FSStats, file_name: str) -> None:
        """Write the given stats to the given file

        :param stats: the stats to write
        :param file_name: the file to write to
        """
        file_path = os.path.join(
            RuntimeManagerConstants.RUNTIME_STATS_DIRECTORY_PATH, file_name
        )

        try:
            # formatted as key-value pairs in TOML
            stats_to_write = (
                f'{RuntimeManagerConstants.RUNTIME_STATS_SCORE_KEY} = "{stats.num_scores}"\n'
                f'{RuntimeManagerConstants.RUNTIME_STATS_RED_CARDS_KEY} = "{stats.num_red_cards}"\n'
                f'{RuntimeManagerConstants.RUNTIME_STATS_YELLOW_CARDS_KEY} = "{stats.num_yellow_cards}"\n'
                f'{RuntimeManagerConstants.RUNTIME_STATS_SHOTS_ON_NET} = "{stats.num_shots_on_net}"\n'
                f'{RuntimeManagerConstants.RUNTIME_STATS_SHOTS_BLOCKED} = "{stats.num_enemy_shots_blocked}"'
            )

            # create temp stats directory if it doesn't exist
            os.makedirs(os.path.dirname(file_path), exist_ok=True)

            with open(file_path, "w") as stats_file:
                stats_file.write(stats_to_write)

        except (FileNotFoundError, PermissionError):
            logging.warning(f"Failed to write TOML FS stats file at: {file_path}")
