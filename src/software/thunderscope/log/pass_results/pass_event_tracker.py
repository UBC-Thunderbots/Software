from software.thunderscope.log.tracker.kick_track import PassTracker
from software.thunderscope.log.pass_results.pass_event import (
    PassEventType,
    TrackedPassEvent,
)
from software.thunderscope.log.trackers.tracked_event import EventType
from typing import override
from dataclasses import dataclass
from software.thunderscope.constants import PassResultsConstants
from proto.import_all_protos import *


@dataclass
class LoggedPass:
    pass_: Pass
    timestamp: float
    score: float


class PassResultTracker(PassTracker):
    def __init__(
        self,
        proto_unix_io: ProtoUnixIO,
        from_team: Team,
        for_team: Team,
        event_queue: queue.Queue,
        **kwargs,
    ):
        """Initializes the PassResultTracker

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

        self.score = 0

        self.logged_passes_map: dict[int, list[NewPass]] = {
            interval: [] for interval in PassResultsConstants.INTERVALS_S
        }

    @override
    def refresh(self) -> None:
        super().refresh()

        self._update_pass_timestamps()

    @override
    def write_event(self, event_type: EventType) -> None:
        """Writes a single event to the event queue of the given type

        :param event_type: the type of event to log
        """
        if not self.curr_pass:
            return

        if not self.cached_world:
            return

        self.logged_passes_map[PassResultsConstants.INTERVALS_S[0]].append(
            LoggedPass(
                pass_=self.curr_pass,
                timestamp=self.cached_world.getMostRecentTimestamp().toSeconds(),
                score=self.score,
            )
        )

    def set_score(score: float) -> None:
        self.score = score

    def _write_pass_event(
        self,
        pass_event_type: PassEventType,
        pass_: Pass,
        score: float = 0.0,
        event_type: EventType = EventType.PASS,
    ) -> None:
        if not self.cached_world:
            return

        tracked_event = get_event_from_world(
            world_msg=self.cached_world_msg,
            event_type=event_type,
            from_team=self.from_team,
            for_team=self.for_team,
        )

        event = TrackedPassEvent(
            tracked_event=tracked_event,
            pass_event_type=pass_event_type,
            pass_=pass_,
            score=score,
        )

        self.event_queue.put(event)

    def _log_if_over_interval(
        logged_pass: NewPass, interval: float, pass_event_type: PassEventType
    ) -> bool:
        if (
            self.cached_world_msg.time_sent.epoch_timestamp_seconds
            - logged_pass.timestamp
            > interval
        ):
            self._write_pass_event(
                pass_event_type=pass_event_type,
                pass_=logged_pass.pass_,
                score=(self.score - logged_pass.score),
            )

            return True

        return False

    def _update_pass_timestamps(self):
        """For all currently logged passes, check if the interval they belong to has passed
        If so, log that pass result
        And move them to the next interval if exists
        """
        pass_event_types = list(map(str, EventType))

        for idx, interval in enumerate(PassResultsConstants.INTERVALS_S):
            logged_passes = self.logged_passes_map[interval]

            pass_event_type = pass_event_types[idx]

            while self.pass_tracker.log_if_over_interval(
                logged_passes[0], interval, pass_event_type
            ):
                logged_pass = logged_passes.pop(0)

                if idx < len(PassResultsConstants.INTERVALS_S) - 1:
                    self.logged_passes_map[
                        PassResultsConstants.INTERVALS_S[idx + 1]
                    ].append(logged_pass)
