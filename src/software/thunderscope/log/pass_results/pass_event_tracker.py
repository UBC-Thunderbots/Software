from software.thunderscope.log.tracker.kick_track import PassTracker
from software.thunderscope.log.pass_results.pass_event import (
    PassEventType,
    TrackedPassEvent,
)
from software.thunderscope.log.trackers.tracked_event import EventType
from typing import override


class PassEventTracker(PassTracker):
    @override
    def write_event(self, event_type: EventType) -> None:
        """Writes a single event to the event queue of the given type

        :param event_type: the type of event to log
        """
        if not self.curr_pass:
            return

        self._write_pass_event(
            pass_event_type=PassEventType.INITIAL,
            event_type=event_type,
            pass_=self.curr_pass,
        )

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

    def log_if_over_interval(
        pass_event: TrackedPassEvent, interval: float, pass_event_type: PassEventType
    ) -> bool:
        if (
            self.cached_world_msg.time_sent.epoch_timestamp_seconds
            - pass_event.tracked_event.timestamp
            > interval
        ):
            self._write_pass_event(
                pass_event_type=pass_event_type,
                pass_=pass_event.pass_,
                score=pass_event.score,
            )

            return True

        return False
