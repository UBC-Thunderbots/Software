from software.thunderscope.proto_unix_io import ProtoUnixIO
from typing import Callable
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from proto.import_all_protos import *
import software.python_bindings as tbots_cpp
import queue
from software.evaluation.logs.event_log import EventType, Team, EventLog


class Tracker:
    """Generic tracker base class. Just tracks the world state."""

    def __init__(
        self,
        proto_unix_io: ProtoUnixIO,
        from_team: Team,
        event_queue: queue.Queue,
        for_team: Team,
        callback: Callable[[EventType], None] = None,
        buffer_size: int = 5,
    ):
        """Initializes the tracker with the given callback and buffer size

        :param proto_unix_io: the proto unix io to get the game state from
        :param from_team: the team that this tracker is tracking from (events are from this team)
        :param for_team: the team that this tracker is tracking for (events are for this team)
                          default is same as the from_team, but can be different
        :param event_queue: the queue to write events to
        :param callback: optional callback to call when there's an event
        :param buffer_size: buffer size for the tracker's io
        """
        self.event_queue = event_queue
        self.buffer_size = buffer_size
        self.from_team = from_team
        self.for_team = for_team
        self.callback = callback

        self.proto_unix_io = proto_unix_io
        self.world_buffer = ThreadSafeBuffer(self.buffer_size, World)
        self.proto_unix_io.register_observer(World, self.world_buffer)

        self.cached_world = None

    def refresh(self) -> None:
        """Refreshes the tracker to get the latest world message"""
        world_msg = self.world_buffer.get(block=False, return_cached=True)

        if world_msg is None:
            return

        self.cached_world_msg = world_msg
        self.cached_world = tbots_cpp.World(world_msg)

        self.refresh_tracker()

    def refresh_tracker(self) -> None:
        pass

    def write_event(self, event_type: EventType) -> None:
        """Writes a single event to the event queue of the given type

        :param event_type: the type of event to log
        """
        if not self.cached_world:
            return

        event = EventLog.from_world(
            world_msg=self.cached_world_msg,
            event_type=event_type,
            from_team=self.from_team,
            for_team=self.for_team,
        )

        self.event_queue.put(event)
