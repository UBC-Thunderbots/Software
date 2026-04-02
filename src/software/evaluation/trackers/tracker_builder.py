from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.evaluation.trackers.tracker import Tracker
from typing import Type, Self
from software.evaluation.trackers.tracked_event import Team
import queue


class TrackerBuilder:
    """Builder class to combine different trackers and update them together"""

    def __init__(
        self,
        proto_unix_io: ProtoUnixIO,
        from_team: Team,
        event_queue: queue.Queue,
        for_team: Team | None = None,
        buffer_size: int = 5,
    ) -> None:
        """Initializes the builder

        :param proto_unix_io: the unix io that the trackers should listen on
        :param from_team: the team that this tracker is tracking from (events are from this team)
        :param for_team: the team that this tracker is tracking for (events are for this team)
                          default is same as the from_team, but can be different
        :param event_queue: the queue to write events to
        """
        self.proto_unix_io = proto_unix_io
        self.from_team = from_team
        self.for_team = from_team if for_team is None else for_team
        self.buffer_size = buffer_size

        self.event_queue = event_queue

        self.trackers = []

    def add_tracker(
        self,
        tracker_cls: Type[Tracker],
        **kwargs,
    ) -> Self:
        """Adds a single tracker to the list

        :param tracker_cls: The class of the tracker to instantiate
        :param **kwargs: tracker-specific arguments
        """
        tracker = tracker_cls(
            proto_unix_io=self.proto_unix_io,
            event_queue=self.event_queue,
            from_team=self.from_team,
            for_team=self.for_team,
            buffer_size=self.buffer_size,
            **kwargs,
        )
        self.trackers.append(tracker)
        return self

    def refresh(self) -> None:
        """Refreshes all the trackers"""
        for tracker in self.trackers:
            tracker.refresh()
