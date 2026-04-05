from software.evaluation.trackers.kick_tracker import PassTracker
from software.evaluation.logs.pass_log import (
    PassLogType,
    PassLog,
)
from software.evaluation.logs.event_log import EventType, Team
from typing import override
from proto.import_all_protos import *
from software.thunderscope.proto_unix_io import ProtoUnixIO
import queue
from software.thunderscope.time_provider import time_provider_instance
import uuid


class PassLogTracker(PassTracker):
    """
    Tracker to keep track of all new passes
    Logs the pass itself + world state at the time of pass
    Plus the world state at specific intervals after that pass
    """
    def __init__(
        self,
        proto_unix_io: ProtoUnixIO,
        from_team: Team,
        event_queue: queue.Queue,
        for_team: Team | None = None,
        **kwargs,
    ):
        """Initializes the PassLogTracker

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

        self.intervals = [item.value for item in PassLogType]
        self.interval_labels = [item for item in PassLogType]

        self.logged_passes_map: dict[int, list[PassLog]] = {
            interval: [] for interval in self.intervals
        }

    @override
    def refresh_tracker(self) -> None:
        """Refreshes the logged passes to log interval states"""

        # IMPORTANT: to refresh and keep track of new passes from parent
        super().refresh_tracker()

        self._update_pass_timestamps()

    @override
    def write_event(self, event_type: EventType) -> None:
        """This method is called by the super-class when there is a new pass
        Adds the pass to the first list of logged passes, at the smallest interval

        :param event_type: (unused)
        """
        if not self.curr_pass:
            return

        pass_result = self._log_pass(
            pass_id=uuid.uuid4(),
            pass_result_type=PassLogType.RESULT_0S,
            pass_=self.curr_pass,
        )

        if not pass_result:
            return

        self.logged_passes_map[self.intervals[0]].append(pass_result)

    def set_score(self, score: float) -> None:
        """Sets the current game score to the given value

        :param score: the new score
        """
        self.score = score

    def _log_pass(
        self, pass_id: uuid.UUID, pass_result_type: PassLogType, pass_: Pass
    ) -> PassLog | None:
        """Logs a single pass, with the given pass and event type

        :param pass_result_type: the result type corresponding to the interval after which
                                we are logging the result
        :param pass_: the pass whose results are being logged
        :param score: the score for this pass's results compared to when the pass started
        """
        if not self.cached_world:
            return None

        pass_result = PassLog.from_world_and_pass(
            pass_id=pass_id,
            pass_result_type=pass_result_type,
            pass_=pass_,
            world_msg=self.cached_world_msg,
            team=self.from_team,
        )

        self.event_queue.put(pass_result)

        return pass_result

    def _log_if_over_interval(
        self, tracked_pass: PassLog, interval: float, pass_result_type: PassLogType
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
            time_provider_instance.elapsed_time_ns() - tracked_pass.get_timestamp()
            > interval
        ):
            self._log_pass(
                pass_id=tracked_pass.pass_id,
                pass_result_type=pass_result_type,
                pass_=tracked_pass.pass_,
            )

            return True

        return False

    def _update_pass_timestamps(self):
        """For all currently logged passes, check if the interval they belong to has passed
        If so, log that pass result
        And move them to the next interval if exists
        """
        for idx, interval in enumerate(self.intervals):
            logged_passes = self.logged_passes_map[interval]

            pass_result_type = self.interval_labels[idx]

            # passes are in the list in chronological order
            while logged_passes and self._log_if_over_interval(
                logged_passes[0], interval, pass_result_type
            ):
                logged_pass = logged_passes.pop(0)

                if idx < len(self.intervals) - 1:
                    self.logged_passes_map[self.intervals[idx + 1]].append(logged_pass)
