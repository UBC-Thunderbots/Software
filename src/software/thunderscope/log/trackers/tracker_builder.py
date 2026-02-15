from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.log.trackers.tracker import Tracker
from typing import Callable, Any, Optional, Type, Self


class TrackerBuilder:
    def __init__(self, proto_unix_io: ProtoUnixIO) -> None:
        self.proto_unix_io = proto_unix_io

        self.trackers = []

    def add_tracker(
        self,
        tracker_cls: Type[Tracker],
        callback: Optional[Callable[[Any], None]] = None,
        **kwargs,
    ) -> Self:
        tracker = tracker_cls(callback=callback, **kwargs)
        tracker.set_proto_unix_io(self.proto_unix_io)
        self.trackers.append(tracker)
        return self

    def refresh(self) -> None:
        for tracker in self.trackers:
            tracker.refresh()
