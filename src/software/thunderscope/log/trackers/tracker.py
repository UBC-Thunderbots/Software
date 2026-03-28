from software.thunderscope.proto_unix_io import ProtoUnixIO
from typing import Callable, Optional, Tuple, Type
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from google.protobuf.message import Message
from proto.import_all_protos import *
import software.python_bindings as tbots_cpp
import os
from software.thunderscope.log.trackers.tracked_event import EventType, get_event_from_world, event_to_csv_row

class Tracker:
    """Generic tracker base class."""

    def __init__(
        self, proto_unix_io: ProtoUnixIO, out_file_path: str, buffer_size: int = 5
    ):
        """Initializes the tracker with the given callback and buffer size
             
        :param proto_unix_io: the proto unix io to get the game state from
        :param out_file_path: the file to write tracked events to               
        :param buffer_size: buffer size for the tracker's io
        """
        self.buffer_size = buffer_size
        
        # create path directories if they doesn't exist
        os.makedirs(os.path.dirname(out_file_path), exist_ok=True)

        self.out_file = open(out_file_path, "w")

        self.proto_unix_io = proto_unix_io
        self.world_buffer = ThreadSafeBuffer(self.buffer_size, World)
        self.proto_unix_io.register_observer(World, self.world_buffer)
        
        self.cached_world = None

    def refresh(self) -> None:
        world_msg = self.world_buffer.get(block=False, return_cached=True)

        if world_msg is None:
            return

        self.cached_world = tbots_cpp.World(world_msg)
    
    def write_event(self, event_type: EventType) -> None:
        if not self.cached_world:
            return
        
        event = get_event_from_world(
            world=self.cached_world, event_type=event_type
        )
        
        csv_row = event_to_csv_row(event)
        
        self.out_file.write(csv_row + "\n")
        self.out_file.flush()
