import socketserver
import time
import base64
import os
from google.protobuf import text_format
from google.protobuf.any_pb2 import Any
from threading import Thread
import queue


class ThreadedUnixListener:
    def __init__(
        self, unix_path, proto_class, max_buffer_size=3, convert_from_any=True
    ):

        """Receive protobuf over unix sockets and buffer them

        :param unix_path: The unix path to receive the new protobuf to plot
        :param proto_class: The type of protobuf we expect to receive
        :param max_buffer_size: The size of the buffer
        :param convert_from_any: Convert from any

        """

        try:
            os.remove(unix_path)
        except:
            pass

        self.server = socketserver.UnixDatagramServer(
            unix_path,
            handler_factory(proto_class, self.__buffer_protobuf, convert_from_any),
        )
        self.stop = False

        self.unix_path = unix_path
        self.proto_buffer = queue.Queue(max_buffer_size)

        self.thread = Thread(target=self.start)
        self.thread.start()

    @property
    def buffer(self):
        return self.proto_buffer

    def maybe_pop(self):
        try:
            return self.proto_buffer.get_nowait()
        except queue.Empty as empty:
            return None

    def __buffer_protobuf(self, proto):
        """Buffer the protobuf, and raise a warning if we overrun the buffer

        :param proto: The protobuf to buffer
        :raises: Warning

        """
        try:
            self.proto_buffer.put_nowait(proto)
        except queue.Full as queue_full:
            pass
            # TODO make this a warning
            # print("ProtoPlotter buffer overrun for {}".format(self.unix_path))

    def serve_till_stopped(self):
        while not self.stop:
            self.server.handle_request()

    def force_stop(self):
        self.stop = True
        self.server.server_close()

    def start(self):
        self.stop = False
        self.serve_till_stopped()


class Session(socketserver.BaseRequestHandler):
    def __init__(self, proto_type, handle_callback, convert_from_any, *args, **keys):
        self.handle_callback = handle_callback
        self.proto_type = proto_type
        self.convert_from_any = convert_from_any
        super().__init__(*args, **keys)

    def handle(self):
        p = base64.b64decode(self.request[0])
        msg = self.proto_type()

        if self.convert_from_any:
            any_msg = Any.FromString(p)
            any_msg.Unpack(msg)
        else:
            msg = self.proto_type.FromString(p)

        self.handle_callback(msg)


def handler_factory(proto_type, handle_callback, convert_from_any):
    def create_handler(*args, **keys):
        return Session(proto_type, handle_callback, convert_from_any, *args, **keys)

    return create_handler
