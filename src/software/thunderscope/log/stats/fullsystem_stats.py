import os

from software.thunderscope.log.trackers import (
    PossessionTracker,
    ShotTracker,
    TrackerBuilder,
    RefereeTracker
)
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from dataclasses import dataclass
from software.thunderscope.proto_unix_io import ProtoUnixIO
import software.python_bindings as tbots_cpp
from software.thunderscope.constants import RuntimeManagerConstants
import logging
from proto.import_all_protos import *
from software.py_constants import ROBOT_MAX_RADIUS_METERS


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


class FullSystemStats:
    # From GoalieTacticConfig
    INCOMING_SHOT_MIN_VELOCITY = 0.2

    def __init__(
        self,
        proto_unix_io: ProtoUnixIO,
        friendly_colour_yellow: bool,
        buffer_size: int = 5,
        record_enemy_stats: bool = False,
    ):
        """Initializes the FullSystem Stats Tracker

        :param friendly_colour_yellow: if the friendly colour is yellow
        :param buffer_size: the buffer size for protocol buffers
        :param record_enemy_stats: if this should record both friendly and enemy stats or just friendly
        """
        self.friendly_colour_yellow = friendly_colour_yellow

        # True if friendly had the last possession, False if enemy
        # None if neither
        self.last_possession_friendly: bool | None = None

        self.world_buffer = ThreadSafeBuffer(buffer_size, World)

        self.stats = FSStats()

        self.tracker = (
            TrackerBuilder(proto_unix_io=proto_unix_io)
            .add_tracker(ShotTracker, callback=self._update_shot_count)
            .add_tracker(PossessionTracker, callback=self._update_posession)
            .add_tracker(RefereeTracker, callback=self._update_referee_info_friendly)
        )

        # the python __del__ destructor isn't called reliably
        # so printing this at the start instead
        print(f"\n\n\n##### Writing FS Stats to {self._get_stats_file()}#####\n\n\n")

        self.record_enemy_stats = record_enemy_stats
        if self.record_enemy_stats:
            self.enemy_stats = FSStats()
            self.tracker = self.tracker.add_tracker(RefereeTracker, callback=self._update_referee_info_enemy)
            print(
                f"\n\n\n##### Writing Enemy FS Stats to {self._get_enemy_stats_file()}#####\n\n\n"
            )

    def refresh(self) -> None:
        """Refreshes the stats for the game so far"""
        world_msg = self.world_buffer.get(block=False, return_cached=True)
        world = tbots_cpp.World(world_msg)

        self.tracker.refresh()

        self._record_goalie_stats(world.ball(), world.field())

        self._record_referee_stats()

        self._flush_stats()

    def _update_shot_count(self, _: Shot):
        self.stats.num_shots_on_net += 1

    def _update_posession(self, friendly_posession: bool | None):
        self.last_possession_friendly = not friendly_posession
        
    def _update_referee_info_friendly(self, num_goals: int, num_yellow_cards: int, num_red_cards: int) -> None:
        self.stats.num_scores = num_goals
        self.stats.num_yellow_cards = num_yellow_cards
        self.stats.num_red_cards = num_red_cards
        
    def _update_referee_info_enemy(self, num_goals: int, num_yellow_cards: int, num_red_cards: int) -> None:
        self.enemy_stats.num_scores = num_goals
        self.enemy_stats.num_yellow_cards = num_yellow_cards
        self.enemy_stats.num_red_cards = num_red_cards

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

    def _get_stats_file(self):
        return os.path.join(
            RuntimeManagerConstants.RUNTIME_STATS_DIRECTORY_PATH,
            RuntimeManagerConstants.RUNTIME_ENEMY_STATS_FILE
            if self.friendly_colour_yellow
            else RuntimeManagerConstants.RUNTIME_FRIENDLY_STATS_FILE,
        )

    def _get_enemy_stats_file(self):
        return os.path.join(
            RuntimeManagerConstants.RUNTIME_STATS_DIRECTORY_PATH,
            RuntimeManagerConstants.RUNTIME_FRIENDLY_FROM_ENEMY_STATS_FILE
            if self.friendly_colour_yellow
            else RuntimeManagerConstants.RUNTIME_ENEMY_FROM_FRIENDLY_STATS_FILE,
        )

    def _flush_stats(self):
        """Write the current stats to disk"""
        stats_file_name = self._get_stats_file()

        self._write_stats_to_file(self.stats, stats_file_name)

        if self.record_enemy_stats:
            enemy_stats_file_name = self._get_enemy_stats_file()

            self._write_stats_to_file(self.enemy_stats, enemy_stats_file_name)

    def _write_stats_to_file(self, stats: FSStats, file_path: str) -> None:
        """Write the given stats to the given file

        :param stats: the stats to write
        :param file_path: the file to write to
        """
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
