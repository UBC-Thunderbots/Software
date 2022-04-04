import logging
import queue
import socket
from threading import Thread

from google.protobuf import text_format
from google.protobuf.any_pb2 import Any


class ThreadedUnixSender:
    def __init__(self, unix_path, max_buffer_size=3):

        """Send protobufs over unix sockets

        :param unix_path: The unix path to send the protobuf to
        :param max_buffer_size: The size of the send buffer. If
                the buffer gets spammed faster than the thread can
                send them out, a buffer overrun msg will be logged.

        """
        self.unix_path = unix_path
        self.proto_buffer = queue.Queue(max_buffer_size)

        self.socket = socket.socket(socket.AF_UNIX, type=socket.SOCK_DGRAM)

        self.stop = False

        # We want to set daemon to true so that the program can exit
        # even if there are still unix listener threads running
        self.thread = Thread(target=self.__send_protobuf, daemon=True)
        self.thread.start()

    def force_stop(self):
        """Stop handling requests
        """
        self.stop = True
        self.server.server_close()

    def __send_protobuf(self):
        """Send the buffered protobuf
        """
        proto = None

        while not self.stop:
            proto = self.proto_buffer.get()
            if proto is not None:
                send = proto.SerializeToString()
                try:
                    self.socket.sendto(send, self.unix_path)
                except Exception:
                    logging.exception("Failed to send on {}".format(self.unix_path))

    def send(self, proto):
        """Buffer a protobuf to be sent by the send thread

        :param proto: The protobuf to send

        """
        try:
            self.proto_buffer.put_nowait(proto)
        except queue.Full as queue_full:
            logging.warning("send buffer overrun for {}".format(self.unix_path))
