import queue
from software.logger.logger import createLogger


class ThreadSafeBuffer(object):

    MIN_DROPPED_BEFORE_LOG = 20

    """Multiple producer, multiple consumer buffer.

               │              buffer_size                 │
               ├──────────────────────────────────────────┤
               │                                          │

               ┌──────┬──────┬──────┬──────┬──────┬───────┐
         put() │      │      │      │      │      │       │  get()
               └──────┴──────┴──────┴──────┴──────┴───────┘
                                           ThreadSafeBuffer

    """

    def __init__(self, buffer_size, protobuf_type, log_overrun=False):

        """A buffer to hold data to be consumed.

        :param buffer size: The size of the buffer.
        :param protobuf_type: To buffer
        :param log_overrun: False

        """
        self.logger = createLogger(protobuf_type.DESCRIPTOR.name + " Buffer")
        self.queue = queue.Queue(buffer_size)
        self.protobuf_type = protobuf_type
        self.log_overrun = log_overrun
        self.cached_msg = protobuf_type()
        self.protos_dropped = 0
        self.last_logged_protos_dropped = 0

    def get(self, block=False, timeout=None, return_cached=True):
        """Get data from the buffer. If the buffer is empty, and
        block is True, wait until a new msg is received. If block
        is False, then return a cached value immediately.

        :param block: If block is True, then block until a new message
                      comes through. Otherwise returned the cached msg.
        :param timeout: If block is True, then wait for this many seconds
        :param return_cached: If queue is empty, decides whether to
                              return cached value (True) or return None (false)

        :return: protobuf (cached if block is False and there is no data
                 in the buffer)

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

        if block:
            return self.queue.get(timeout=timeout)

        try:
            self.cached_msg = self.queue.get_nowait()

        except queue.Empty as empty:
            if not return_cached:
                return None

        return self.cached_msg

    def put(self, proto, block=False, timeout=None):
        """Put data into the buffer. If the buffer is full, then
        the proto will be logged.

        :param proto: The proto to place in the buffer
        :param block: Should block until there is space in the buffer
        :param timeout: If block is True, then wait for this many seconds

        """
        if block:
            self.queue.put(proto, block, timeout)
            return

        try:
            self.queue.put_nowait(proto)
        except queue.Full as full:
            self.protos_dropped += 1
