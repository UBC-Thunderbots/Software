import base64
import logging
import os
import queue
import socketserver
from threading import Thread

from google.protobuf import text_format
from google.protobuf.any_pb2 import Any
from proto.robot_log_msg_pb2 import RobotLog


class ThreadedUnixListener:
    def __init__(
        self, unix_path, proto_class, max_buffer_size=3, convert_from_any=True
    ):

        """Receive protobuf over unix sockets and buffers them

        :param unix_path: The unix path to receive the new protobuf to plot
        :param proto_class: The type of protobuf we expect to receive
        :param max_buffer_size: The size of the buffer
        :param convert_from_any: Convert from any

        """

        # cleanup the old path if it exists
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

        self.thread = Thread(target=self.start, daemon=True)
        self.thread.start()

    @property
    def buffer(self):
        return self.proto_buffer

    def maybe_pop(self):
        """Pop from the buffer if a new packet exists. If not just return None

        :return: proto if exists, else None
        :rtype: proto or None

        """
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
            logging.warning("receive buffer overrun for {}".format(self.unix_path))

    def serve_till_stopped(self):
        """Keep handling requests until force_stop is called
        """
        while not self.stop:
            self.server.handle_request()

    def force_stop(self):
        """Stop handling requests
        """
        self.stop = True
        self.server.server_close()

    def start(self):
        """Start handling requests
        """
        self.stop = False
        self.serve_till_stopped()


class Session(socketserver.BaseRequestHandler):
    def __init__(self, proto_type, handle_callback, convert_from_any, *args, **keys):
        self.handle_callback = handle_callback
        self.proto_type = proto_type
        self.convert_from_any = convert_from_any
        super().__init__(*args, **keys)

    def handle(self):
        """Decode the base64 request and unpack from Any if we are receiving
        an Any protobuf. If not, just unpack directly into the type provided.

        Then, trigger the handle callback

        """
        if self.convert_from_any:
            packet = base64.b64decode(self.request[0])
            msg = self.proto_type()
            any_msg = Any.FromString(packet)
            any_msg.Unpack(msg)
        else:
            packet = None
            if self.proto_type == RobotLog:
                packet = base64.b64decode(self.request[0])
            else:
                packet = self.request[0]
            msg = self.proto_type.FromString(packet)

        self.handle_callback(msg)


def handler_factory(proto_type, handle_callback, convert_from_any):
    """To pass in an arbitrary handle callback into the SocketServer,
    we need to create a constructor that can create a Session object with
    appropriate handle function.

    :param proto_type: The type of protobuf to handle
    :param handle_callback: The callback to run
    :param convert_from_any: If true, the message needs to be decoded
                             into Any before into the proto_type
    """

    def create_handler(*args, **keys):
        return Session(proto_type, handle_callback, convert_from_any, *args, **keys)

    return create_handler
