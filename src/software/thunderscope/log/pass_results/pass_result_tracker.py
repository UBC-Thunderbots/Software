from software.thunderscope.log.trackers.kick_tracker import PassTracker
from software.thunderscope.log.pass_results.pass_event import (
    PassResultType,
    TrackedPassResult,
)
from software.thunderscope.log.trackers.tracked_event import (
    EventType,
    Team,
    get_event_from_world,
)
from typing import override
from dataclasses import dataclass
from software.thunderscope.constants import PassResultsConstants
from proto.import_all_protos import *
from software.thunderscope.proto_unix_io import ProtoUnixIO
import queue


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
        event_queue: queue.Queue,
        for_team: Team | None = None,
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
            for_team=for_team if for_team is not None else from_team,
            event_queue=event_queue,
            **kwargs,
        )

        self.score = 0

        self.logged_passes_map: dict[int, list[LoggedPass]] = {
            interval: [] for interval in PassResultsConstants.INTERVALS_S
        }

    @override
    def refresh(self) -> None:
        super().refresh()

        self._update_pass_timestamps()

    @override
    def write_event(self, event_type: EventType) -> None:
        """This method is called by the super-class when there is a new pass
        Adds the pass to the first list of logged passes, at the smallest interval

        :param event_type: (unused)
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

    def set_score(self, score: float) -> None:
        """Sets the current game score to the given value

        :param score: the new score
        """
        self.score = score

    def _log_pass_result(
        self, pass_result_type: PassResultType, pass_: Pass, score: float = 0.0
    ) -> None:
        """Logs a single pass result, with the given pass, score, and event type

        :param pass_result_type: the result type corresponding to the interval after which
                                we are logging the result
        :param pass_: the pass whose results are being logged
        :param score: the score for this pass's results compared to when the pass started
        """
        if not self.cached_world:
            return

        pass_event = get_event_from_world(
            world_msg=self.cached_world_msg,
            event_type=EventType.PASS,
            from_team=self.from_team,
            for_team=self.for_team,
        )

        event = TrackedPassResult(
            pass_event=pass_event,
            pass_result_type=pass_result_type,
            pass_=pass_,
            score=score,
        )

        self.event_queue.put(event)

    def _log_if_over_interval(
        self, logged_pass: LoggedPass, interval: float, pass_result_type: PassResultType
    ) -> bool:
        """Checks if the given pass we logged is older than the given interval
        If so, logs a result with the given type with the correct score

        :param logged_pass: the pass we previously logged, with the timestamp and score at the time
        :param interval: the interval that the pass may be older than
        :param pass_result_type: the result type corresponding to this interval
        :return: True if the pass is older than interval (and therefore result has been logged)
                 False if not
        """
        if (
            self.cached_world_msg.time_sent.epoch_timestamp_seconds
            - logged_pass.timestamp
            > interval
        ):
            self._log_pass_result(
                pass_result_type=pass_result_type,
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
        pass_result_types = list(PassResultType)

        for idx, interval in enumerate(PassResultsConstants.INTERVALS_S):
            logged_passes = self.logged_passes_map[interval]

            pass_result_type = pass_result_types[idx]

            # passes are in the list in chronological order
            while self._log_if_over_interval(
                logged_passes[0], interval, pass_result_type
            ):
                logged_pass = logged_passes.pop(0)

                if idx < len(PassResultsConstants.INTERVALS_S) - 1:
                    self.logged_passes_map[
                        PassResultsConstants.INTERVALS_S[idx + 1]
                    ].append(logged_pass)
