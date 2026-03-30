from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.log.trackers.tracker import Tracker
from typing import Callable, Any, Optional, Type, Self


class TrackerBuilder:
<<<<<<< HEAD
    def __init__(self, proto_unix_io: ProtoUnixIO) -> None:
=======
    """Builder class to combine different trackers and update them together"""

    def __init__(self, proto_unix_io: ProtoUnixIO) -> None:
        """Initializes the builder

        :param proto_unix_io: the unix io that the trackers should listen on
        """
>>>>>>> 2d65fc71c3016c5d9626754a7d5e5b30ab3395ae
        self.proto_unix_io = proto_unix_io

        self.trackers = []

    def add_tracker(
        self,
        tracker_cls: Type[Tracker],
        callback: Optional[Callable[[Any], None]] = None,
<<<<<<< HEAD
        buffer_size: int = 5,
        **kwargs,
    ) -> Self:
=======
        **kwargs,
    ) -> Self:
        """Adds a single tracker to the list

        :param tracker_cls: The class of the tracker to instantiate
        :param callback: function that the tracker should call when it tracks an event
        :param **kwargs: tracker-specific arguments
        """
>>>>>>> 2d65fc71c3016c5d9626754a7d5e5b30ab3395ae
        tracker = tracker_cls(callback=callback, **kwargs)
        tracker.set_proto_unix_io(self.proto_unix_io)
        self.trackers.append(tracker)
        return self

    def refresh(self) -> None:
<<<<<<< HEAD
=======
        """Refreshes all the trackers"""
>>>>>>> 2d65fc71c3016c5d9626754a7d5e5b30ab3395ae
        for tracker in self.trackers:
            tracker.refresh()
