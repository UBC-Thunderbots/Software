from software.thunderscope.proto_unix_io import ProtoUnixIO
from typing import Callable, Optional, Tuple, Type
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from google.protobuf.message import Message


class Tracker:
    def __init__(
        self, callback: Optional[Callable[..., None]] = None, buffer_size: int = 5
    ):
        self.callback = callback
        self.buffer_size = buffer_size

    def set_proto_unix_io(
        self,
        proto_unix_io: ProtoUnixIO,
        type_buffers: list[Tuple[Type[Message], ThreadSafeBuffer]],
    ) -> None:
        for type_buffer in type_buffers:
            proto_unix_io.register_observer(*type_buffer)

    def refresh(self) -> None:
        raise Exception("Not Implemented, please use the appropriate subclass!")
