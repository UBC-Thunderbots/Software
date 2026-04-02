from typing import override
from software.evaluation.trackers.tracker import Tracker
from proto.import_all_protos import *
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.evaluation.trackers.tracked_event import EventType, Team
import queue


class RefereeTracker(Tracker):
    """Tracks Referee events, like goals and yellow / red cards for the friendly team only"""

    def __init__(
        self,
        proto_unix_io: ProtoUnixIO,
        from_team: Team,
        for_team: Team,
        event_queue: queue.Queue,
        friendly_color_yellow: bool,
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

        self.num_yellow_cards = 0
        self.num_red_cards = 0
        self.num_goals = 0

    @override
    def refresh(self):
        """Refresh and log the latest referee information"""
        refree_msg = self.referee_buffer.get(block=False, return_cached=True)

        if not refree_msg:
            return

        if refree_msg.HasField("yellow" if self.friendly_color_yellow else "blue"):
            team_info = (
                refree_msg.yellow if self.friendly_color_yellow else refree_msg.blue
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
        if new_value != old_value:
            self.write_event(event_type=event_type)

        return new_value
