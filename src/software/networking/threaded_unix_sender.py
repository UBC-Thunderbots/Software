import logging
import queue
import socket
from threading import Thread

from google.protobuf import text_format
from google.protobuf.any_pb2 import Any
from google.protobuf.message import EncodeError
from software.py_constants import UNIX_BUFFER_SIZE
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class ThreadedUnixSender:

    MAX_SEND_FAILURES_BEFORE_LOG = 100

    def __init__(self, unix_path, proto_type, max_buffer_size=3):

        """Send protobufs over unix sockets

        :param unix_path: The unix path to send the protobuf to
        :param max_buffer_size: The size of the send buffer. If
                the buffer gets spammed faster than the thread can
                send them out, a buffer overrun msg will be logged.

        """
        self.unix_path = unix_path
        self.proto_buffer = ThreadSafeBuffer(max_buffer_size, proto_type)

        self.socket = socket.socket(socket.AF_UNIX, type=socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, UNIX_BUFFER_SIZE)

        self.stop = False

        # We want to set daemon to true so that the program can exit
        # even if there are still unix listener threads running
        self.thread = Thread(target=self.__send_protobuf, daemon=True)
        self.thread.start()
        self.send_failures = 0

    def force_stop(self):
        """Stop handling requests
        """
        self.stop = True
        self.socket.close()

    def __send_protobuf(self):
        """Send the buffered protobuf
        """
        proto = None

        while not self.stop:
            proto = self.proto_buffer.get(block=True, return_cached=False)
            if proto is not None:
                try:
                    send = proto.SerializeToString()
                    self.socket.sendto(send, self.unix_path)
                except EncodeError:
                    logging.warning(
                        "Received an invalid proto of type {}".format(
                            proto.DESCRIPTOR.full_name
                        )
                    )
                    self.send_failures = 0
                except Exception:
                    self.send_failures += 1
                    if (
                        self.send_failures
                        > ThreadedUnixSender.MAX_SEND_FAILURES_BEFORE_LOG
                    ):
                        logging.warning(
                            "Failed to send on {}, make sure the receiver is running".format(
                                self.unix_path
                            )
                        )
                        self.send_failures = 0

    def send(self, proto):
        """Buffer a protobuf to be sent by the send thread

        :param proto: The protobuf to send

        """
        try:
            self.proto_buffer.put(proto)
        except queue.Full as queue_full:
            logging.warning("send buffer overrun for {}".format(self.unix_path))
