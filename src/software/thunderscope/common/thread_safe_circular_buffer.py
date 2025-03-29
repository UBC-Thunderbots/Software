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
        """A circular buffer to hold data to be consumed.

        :param buffer_size: The max size of the buffer.
        :param protobuf_type: To buffer
        :param log_overrun: If True, warns when we lose protos during future operations
        """
        super().__init__(buffer_size, protobuf_type)
        self.log_overrun = log_overrun
        self.buffer = deque(maxlen=buffer_size)
        self.empty_exception = IndexError

    @override
    def get(
            self, block: bool = False, timeout: float = None, return_cached: bool = True
    ) -> Optional[Message]:
        """Get data from the buffer immediately.

        If the buffer is empty:
            - Return cached message if return_cached is True, otherwise returns None

        :param block: This does nothing as all operations are immediate
        :param timeout: This does nothing as all operations are immediate
        :param return_cached: If buffer is empty, decides whether to
                              return cached value (True) or return None (false)
        :return: protobuf (cached if there is no data in the buffer and return_cached is True)
        """
        if (
                self.log_overrun
                and self.protos_dropped > self.last_logged_protos_dropped
                and self.protos_dropped > self.MIN_DROPPED_BEFORE_LOG
        ):
            self.logger.warn(
                "packets dropped; thunderscope did not show {} protos".format(
                    self.protos_dropped
                )
            )
            self.last_logged_protos_dropped = self.protos_dropped

        try:
            self.cached_msg = self.buffer.popleft()
        except self.empty_exception:
            if not return_cached:
                return None

        return self.cached_msg

    @override
    def put(self, proto: Message, block: bool = False, timeout: float = None) -> None:
        """Put data into the buffer. If the buffer is full, the proto may be logged.

        :param proto: The proto to place in the buffer
        :param block: True blocks overwriting items in a full buffer, dropping the proto. False writes every time
        :param timeout: This does nothing as all operations are immediate
        """
        if len(self.buffer) == self.buffer.maxlen:
            self.protos_dropped += 1
            if block:
                return
        self.buffer.append(proto)

    @override
    def size(self) -> int:
        """Returns the number of objects in the buffer"""
        return len(self.buffer)
