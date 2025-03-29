from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from collections import deque
from software.logger.logger import create_logger
from typing import Type, Optional
from google.protobuf.message import Message
from typing import override


class ThreadSafeCircularBuffer(ThreadSafeBuffer):
    """Multiple producer, multiple consumer buffer. See: ThreadSafeBuffer

               │              buffer_size                 │
               ├──────────────────────────────────────────┤
               │                                          │

               ┌──────┬──────┬──────┬──────┬──────┬───────┐
         put() │      │      │      │      │      │       │  get()
               └──────┴──────┴──────┴──────┴──────┴───────┘
                                    ThreadSafeCircularBuffer
    """

    def __init__(
            self, buffer_size: int, protobuf_type: Type[Message], log_overrun: bool = False
    ) -> None:
        """A buffer to hold data to be consumed.

        :param buffer size: The size of the buffer.
        :param protobuf_type: To buffer
        :param log_overrun: False
        """
        super().__init__(buffer_size, protobuf_type)
        self.log_overrun = log_overrun
        self.buffer = deque(maxlen=buffer_size)
        self.empty_exception = IndexError

    @override
    def get(
            self, block: bool = False, timeout: float = None, return_cached: bool = True
    ) -> Optional[Message]:
        """Get data from the buffer.

        If the buffer is empty:

            - If block is True
                - wait until a new msg is received.
                - If a timeout is supplied, wait for timeout seconds
                - Then throw an error, or return cached message if return_cached is True

            - If block is False
                - Return None if return_cached is False
                - Return cached message if return_cached is True

        :param block: If block is True, then block until a new message
                      comes through, or returned the cached msg if return_cached = True
        :param timeout: If block is True, then wait for this many seconds before
                        throwing an error or returning cached
        :param return_cached: If buffer is empty, decides whether to
                              return cached value (True) or return None / throw an error (false)
        :return: protobuf (cached if block is False and there is no data
                 in the buffer)
        """
        try:
            self.cached_msg = self.buffer.popleft()
        except self.empty_exception:
            if not return_cached:
                if block:
                    raise super().empty_exception
                else:
                    return None

        return self.cached_msg

    @override
    def put(self, proto: Message, block: bool = False, timeout: float = None) -> None:
        """Put data into the buffer. If the buffer is full, then
        the proto will be logged.

        :param proto: The proto to place in the buffer
        :param block: Should block until there is space in the buffer
        :param timeout: If block is True, then wait for this many seconds
        """
        if len(self.buffer) == self.buffer.maxlen:
            self.protos_dropped += 1
        self.buffer.append(proto)

    @override
    def size(self) -> int:
        """Returns the number of objects in the buffer"""
        return len(self.buffer)
    