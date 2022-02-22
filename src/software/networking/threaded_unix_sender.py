import socketserver
import socket
import time
import base64
import os
from google.protobuf import text_format
from google.protobuf.any_pb2 import Any
from threading import Thread
import logging
import queue


class ThreadedUnixSender:
    def __init__(self, unix_path, max_buffer_size=3, convert_to_any=True):

        """Send protobufs over unix sockets

        :param unix_path: The unix path to send the protobuf to
        :param max_buffer_size: The size of the send buffer
        :param convert_from_any: Convert from any

        """
        self.unix_path = unix_path
        self.proto_buffer = queue.Queue(max_buffer_size)

        self.socket = socket.socket(socket.AF_UNIX, type=socket.SOCK_DGRAM)

        self.thread = Thread(target=self.__send_protobuf)
        self.thread.start()

    @property
    def buffer(self):
        return self.proto_buffer

    def __send_protobuf(self):
        """Send the buffered protobuf
        """
        proto = None

        while True:
            proto = self.proto_buffer.get()
            if proto is not None:
                send = proto.SerializeToString()
                self.socket.sendto(send, self.unix_path)

    def send(self, proto):
        """Buffer a protobuf to be sent by the send thread

        :param proto: The protobuf to send

        """
        try:
            self.proto_buffer.put_nowait(proto)
        except queue.Full as queue_full:
            logging.warning("send buffer overrun for {}".format(self.unix_path))
