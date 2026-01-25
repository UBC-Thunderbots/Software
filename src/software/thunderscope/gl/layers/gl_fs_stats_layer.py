import os
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.gl.layers.gl_layer import GLLayer
from dataclasses import dataclass
import software.python_bindings as tbots_cpp
from software.thunderscope.constants import RuntimeManagerConstants
import logging
from proto.visualization_pb2 import AttackerVisualization, GoalieVisualization
from proto.import_all_protos import *
from software.py_constants import ROBOT_MAX_RADIUS_METERS
from typing import override


@dataclass
class FSStats:
    num_yellow_cards: int = 0
    num_red_cards: int = 0
    num_scores: int = 0

    num_shots_on_net: int = 0
    latest_shot_on_net: tbots_cpp.Point = None

    is_shot_incoming: bool = False
    num_enemy_shots_blocked: int = 0


class GlFSStatsLayer(GLLayer):
    # From GoalieTacticConfig
    INCOMING_SHOT_MIN_VELOCITY = 0.2

    def __init__(
        self,
        name: str,
        friendly_colour_yellow: bool,
        buffer_size: int = 5,
        record_enemy_stats: bool = False,
    ):
        super().__init__(name)

        self.friendly_colour_yellow = friendly_colour_yellow

        # True if enemy had the last possession, False if friendly
        # None if neither
        self.last_possession_enemy: bool | None = None

        self.attacker_vis_buffer = ThreadSafeBuffer(buffer_size, AttackerVisualization)
        self.goalie_vis_buffer = ThreadSafeBuffer(buffer_size, GoalieVisualization)
        self.referee_buffer = ThreadSafeBuffer(buffer_size, Referee)
        self.world_buffer = ThreadSafeBuffer(buffer_size, World)

        self.stats = FSStats()

        self.record_enemy_stats = record_enemy_stats
        if self.record_enemy_stats:
            self.enemy_stats = FSStats()

    @override
    def refresh_graphics(self) -> None:
        """Refreshes the stats for the game so far"""
        self._record_attacker_stats()

        self._record_goalie_stats()

        self._record_world_stats()

        self._record_referee_stats()

    def _record_world_stats(self) -> None:
        world_msg = self.world_buffer.get(block=False, return_cached=True)

        if not world_msg.HasField("friendly_team") and not world_msg.HasField(
            "enemy_team"
        ):
            return

        world = tbots_cpp.World(world_msg)

        friendly_team, enemy_team = (
            (world.enemyTeam, world.friendlyTeam)
            if self.friendly_color_yellow
            else (world.friendlyTeam, world.enemyTeam)
        )

        if self.record_enemy_stats:
            self._record_enemy_blocked_goals(world.ball().position(), world.field())

        self.last_possession_enemy = self._check_posession_for_teams(
            friendly_team, enemy_team, world.ball().position()
        )

    def _record_enemy_blocked_goals(
        self, ball: tbots_cpp.Ball, field: tbots_cpp.Field
    ) -> None:
        """HEAVY WORKAROUND: as a fallback for when the enemy fullsystem doesn't calculate this
        This should ordinarily be calculated by using the Goalie Visualization proto on the enemy side
        not very clean, but follows similar logic to goalie_fsm.cpp, but from the enemy's perspective
        """
        ball_ray = tbots_cpp.Ray(ball.position(), ball.velocity())
        enemy_goal_segment = tbots_cpp.Segment(
            field.enemyGoalpostPos() + tbots_cpp.Vector(0, -ROBOT_MAX_RADIUS_METERS),
            field.enemyGoalpostNeg() + tbots_cpp.Vector(0, ROBOT_MAX_RADIUS_METERS),
        )
        shot_incoming_for_enemy = (
            len(tbots_cpp.intersects(ball_ray, enemy_goal_segment)) != 0
            and ball.velocity().length() > self.INCOMING_SHOT_MIN_VELOCITY
        )

        if not shot_incoming_for_enemy and self.enemy_stats.is_shot_incoming:
            self.enemy_stats.num_enemy_shots_blocked += 1

        self.enemy_stats.is_shot_incoming = shot_incoming_for_enemy

    def _check_posession_for_teams(
        self,
        friendly_team: tbots_cpp.Team,
        enemy_team: tbots_cpp.Team,
        ball_position: tbots_cpp.Point,
    ) -> bool | None:
        if self._check_posession_for_team(enemy_team, ball_position):
            return True

        if self._check_posession_for_team(friendly_team, ball_position):
            return False

        return None

    def _check_posession_for_team(
        self, team: tbots_cpp.Team, ball_position: tbots_cpp.Point
    ):
        for robot in team.getAllRobots():
            if robot.isNearDribbler(ball_position):
                return True

        return False

    def _record_attacker_stats(self) -> None:
        attacker_vis_msg = self.attacker_vis_buffer.get(block=False, return_cached=True)

        if attacker_vis_msg and attacker_vis_msg.HasField("shot"):
            if attacker_vis_msg.shot.HasField("shot_origin"):
                shot_origin = attacker_vis_msg.shot.shot_origin
                shot_origin_point = tbots_cpp.Point(
                    shot_origin.x_meters, shot_origin.y_meters
                )

                if (
                    self.stats.latest_shot_on_net is None
                    or self.stats.latest_shot_on_net != shot_origin_point
                ):
                    self.stats.latest_shot_on_net = shot_origin_point
                    self.stats.num_shots_on_net += 1

    def _record_goalie_stats(self) -> None:
        goalie_vis_msg = self.goalie_vis_buffer.get(block=False)

        if goalie_vis_msg and goalie_vis_msg.HasField("is_shot_incoming"):
            shot_incoming = (
                goalie_vis_msg.is_shot_incoming and self.last_possession_enemy
            )

            # assume that if a ball was previously incoming and then was no longer incoming
            # then we successfully blocked it
            # TODO: verify if this is accurate
            if not goalie_vis_msg.is_shot_incoming and self.stats.is_shot_incoming:
                self.stats.num_enemy_shots_blocked += 1

            # if there wasn't an incoming shot and now there is, the enemy must have taken a shot at us
            if (
                self.record_enemy_stats
                and shot_incoming
                and not self.stats.is_shot_incoming
            ):
                self.enemy_stats.num_shots_on_net += 1

            self.stats.is_shot_incoming = shot_incoming

    def _record_referee_stats(self) -> None:
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

    def _record_referee_stats_per_team(self, team_info: TeamInfo, stats: FSStats):
        if team_info.HasField("score"):
            stats.num_scores = team_info.score

        if team_info.HasField("yellow_cards"):
            stats.num_yellow_cards = team_info.yellow_cards

        if team_info.HasField("red_cards"):
            stats.num_red_cards = team_info.red_cards

    def __del__(self):
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
