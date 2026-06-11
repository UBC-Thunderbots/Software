from software.evaluation.trackers.tracker import Tracker
from typing import override
from software.thunderscope.proto_unix_io import ProtoUnixIO
from proto.import_all_protos import *
import software.python_bindings as tbots_cpp
from software.evaluation.logs.event_log import EventType, Team
import queue
from software.py_constants import BALL_TO_FRONT_OF_ROBOT_DISTANCE_WHEN_DRIBBLING


class PossessionTracker(Tracker):
    """Tracker to track and log when ball possession changes"""

    def __init__(
        self,
        proto_unix_io: ProtoUnixIO,
        from_team: Team,
        for_team: Team,
        event_queue: queue.Queue,
        **kwargs,
    ):
        """Initializes the PossessionTracker

        :param proto_unix_io: the proto unix io to get the game state from
        :param from_team: the team that this tracker is tracking from (events are from this team)
        :param for_team: the team that this tracker is tracking for (events are for this team)
                          default is same as the from_team, but can be different
        :param event_queue: the queue to write events to
        """
        super().__init__(
            proto_unix_io=proto_unix_io,
            from_team=from_team,
            for_team=for_team,
            event_queue=event_queue,
            **kwargs,
        )

        # start with no team having possession
        self.curr_possession = None

    @override
    def refresh_tracker(self) -> None:
        """Refresh and logs any changes in ball possession"""
        if self.cached_world is None:
            return

        self._log_posession_for_friendly(
            self.cached_world.friendlyTeam(),
            self.cached_world.enemyTeam(),
            self.cached_world.ball().position(),
        )

    def _log_posession_for_friendly(
        self,
        friendly_team: tbots_cpp.Team,
        enemy_team: tbots_cpp.Team,
        ball_position: tbots_cpp.Point,
    ) -> None:
        """Detects possession changes and logs the corresponding start/end events

        Checks the current world state against the last known possession.
        True if friendly possession, False is enemy possession, None if neither

        If a transition occurs (e.g., Friendly -> None, None -> Enemy), it triggers
        a write_event with the appropriate EventType and updates the internal
        possession state.

        :param friendly_team: the friendly team object
        :param enemy_team: the enemy team object
        :param ball_position: the current position of the ball
        :return: None
        """
        new_possession = None

        if self._check_posession_for_team(friendly_team, ball_position):
            new_possession = True
        elif self._check_posession_for_team(enemy_team, ball_position):
            new_possession = False

        # if possession didn't change, no need to log
        if new_possession == self.curr_possession:
            return

        # mark the end of the last possession since it has changed
        if self.curr_possession:
            self.write_event(event_type=EventType.FRIENDLY_POSSESSION_END)
        elif self.curr_possession == False:
            self.write_event(event_type=EventType.ENEMY_POSSESSION_END)

        # log the start of the new, changed possession
        if new_possession:
            self.write_event(event_type=EventType.FRIENDLY_POSSESSION_START)
        elif new_possession == False:
            self.write_event(event_type=EventType.ENEMY_POSSESSION_START)

        self.curr_possession = new_possession

    def _check_posession_for_team(
        self, team: tbots_cpp.Team, ball_position: tbots_cpp.Point
    ) -> bool:
        """Check if the given team has possession of the ball

        :param team: the team to check
        :param ball_position: the current ball position
        :return: True if the team has possession, False otherwise
        """
        for robot in team.getAllRobots():
            # higher tolerance to make possession a bit stickier
            if robot.isNearDribbler(
                ball_position, BALL_TO_FRONT_OF_ROBOT_DISTANCE_WHEN_DRIBBLING * 2
            ):
                return True

        return False
