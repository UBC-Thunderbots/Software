from software.thunderscope.proto_unix_io import ProtoUnixIO
from typing import Callable, Optional, Tuple, Type
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from google.protobuf.message import Message


class Tracker:
    """Generic tracker base class."""

    def __init__(
        self, callback: Optional[Callable[..., None]] = None, buffer_size: int = 5
    ):
        """Initializes the tracker with the given callback and buffer size

        :param callback: the function to call when the tracker tracks an event
        :param buffer_size: buffer size for the tracker's io
        """
        self.callback = callback
        self.buffer_size = buffer_size

    def set_proto_unix_io(
        self,
        proto_unix_io: ProtoUnixIO,
        type_buffers: list[Tuple[Type[Message], ThreadSafeBuffer]],
    ) -> None:
        """Registers the given message types and buffers to the given proto unix io connection

        :param proto_unix_io: the io connection to listen on
        :param type_buffers: a list of (Message Type, Buffer) tuples.
                             messages of each type will be placed into their corresponding buffer
        """
        for message_type, buffer in type_buffers:
            proto_unix_io.register_observer(message_type, buffer)

    def refresh(self) -> None:
        raise Exception("Not Implemented, please use the appropriate subclass!")
