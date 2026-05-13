from typing import override, Callable
from software.evaluation.trackers.tracker import Tracker
from proto.import_all_protos import *
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.evaluation.logs.event_log import EventType, Team
import queue


class RefereeTracker(Tracker):
    """Tracks Referee events, like goals and yellow / red cards for the friendly team only"""

    # we want to ignore all breaks, times before the game actually starts
    # and all penalty related stages
    STAGES_TO_IGNORE = [
        Referee.Stage.PENALTY_SHOOTOUT_BREAK,
        Referee.Stage.PENALTY_SHOOTOUT,
        Referee.Stage.NORMAL_FIRST_HALF_PRE,
        Referee.Stage.NORMAL_SECOND_HALF_PRE,
        Referee.Stage.EXTRA_TIME_BREAK,
        Referee.Stage.EXTRA_FIRST_HALF_PRE,
        Referee.Stage.EXTRA_SECOND_HALF_PRE,
    ]

    def __init__(
        self,
        proto_unix_io: ProtoUnixIO,
        from_team: Team,
        for_team: Team,
        event_queue: queue.Queue,
        friendly_color_yellow: bool,
        toggle_logging: Callable[[bool], None] | None = None,
        **kwargs,
    ):
        """Initializes the RefereeTracker

        :param proto_unix_io: the proto unix io to get the game state from
        :param from_team: the team that this tracker is tracking from (events are from this team)
        :param for_team: the team that this tracker is tracking for (events are for this team)
                          default is same as the from_team, but can be different
        :param event_queue: the queue to write events to
        :param friendly_color_yellow: if the friendly color is yellow or blue
        """
        super().__init__(
            proto_unix_io=proto_unix_io,
            from_team=from_team,
            for_team=for_team,
            event_queue=event_queue,
            **kwargs,
        )

        self.referee_buffer = ThreadSafeBuffer(self.buffer_size, Referee)
        self.proto_unix_io.register_observer(Referee, self.referee_buffer)

        self.friendly_color_yellow = friendly_color_yellow

        # we can use this callback to turn on / off logging
        # during stages we don't care about
        self.toggle_logging = toggle_logging

        self.num_yellow_cards = 0
        self.num_red_cards = 0
        self.num_goals = 0

        self.curr_stage = None

    @override
    def refresh_tracker(self) -> None:
        """Refresh and log the latest referee information"""
        referee_msg = self.referee_buffer.get(block=False, return_cached=True)

        if not referee_msg:
            return

        game_stage = referee_msg.stage

        if game_stage in self.STAGES_TO_IGNORE:
            if self.toggle_logging:
                self.toggle_logging(False)
            return

        if self.toggle_logging:
            self.toggle_logging(True)

        # if game has just started, log a game start event once
        if game_stage == Referee.Stage.NORMAL_FIRST_HALF:
            self.curr_stage = self._log_event_if_change(
                new_value=game_stage,
                old_value=self.curr_stage,
                event_type=EventType.GAME_START,
            )
            return

        # if the game has just ended, log a game end event once
        if game_stage == Referee.Stage.POST_GAME:
            self.curr_stage = self._log_event_if_change(
                new_value=game_stage,
                old_value=self.curr_stage,
                event_type=EventType.GAME_END,
            )
            return

        self.curr_stage = game_stage

        if referee_msg.HasField("yellow" if self.friendly_color_yellow else "blue"):
            team_info = (
                referee_msg.yellow if self.friendly_color_yellow else referee_msg.blue
            )

            if team_info.HasField("score"):
                self.num_goals = self._log_event_if_change(
                    team_info.score, self.num_goals, EventType.GOAL_SCORED
                )

            if team_info.HasField("yellow_cards"):
                self.num_yellow_cards = self._log_event_if_change(
                    team_info.yellow_cards, self.num_yellow_cards, EventType.YELLOW_CARD
                )

            if team_info.HasField("red_cards"):
                self.num_red_cards = self._log_event_if_change(
                    team_info.red_cards, self.num_red_cards, EventType.RED_CARD
                )

    def _log_event_if_change(
        self, new_value: int, old_value: int, event_type: EventType
    ) -> int:
        """
        Logs an event of the given type if the given value has changed between old and new
        
        :param new_value: the new value
        :param old_value: the old value to compare with
        :param event_type: the type of event to log if a change is detected
        :return: the new value unchanged
        """
        if new_value != old_value:
            self.write_event(event_type=event_type)

        return new_value
