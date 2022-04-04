import base64
import os
import queue
import socketserver
from threading import Thread
from software.logger.logger import createLogger

logger = createLogger(__name__)

import proto
from google.protobuf.any_pb2 import Any


class ThreadedUnixListener:
    def __init__(
        self, unix_path, proto_class=None, base64_encoded=False, max_buffer_size=100
    ):

        """Receive protobuf over unix sockets and buffers them

        :param unix_path: The unix path to receive the new protobuf to plot
        :param proto_class: The protobuf to unpack from (None if its encoded in the payload)
        :param max_buffer_size: The size of the buffer
        :param base64_encoded: False

        """

        # cleanup the old path if it exists
        try:
            os.remove(unix_path)
        except:
            pass

        self.server = socketserver.UnixDatagramServer(
            unix_path,
            handler_factory(self.__buffer_protobuf, proto_class, base64_encoded),
        )
        self.stop = False

        self.unix_path = unix_path
        self.proto_buffer = queue.Queue(max_buffer_size)

        # We want to set daemon to true so that the program can exit
        # even if there are still unix listener threads running
        self.thread = Thread(target=self.start, daemon=True)
        self.thread.start()

    @property
    def buffer(self):
        return self.proto_buffer

    def get_most_recent_message(self, block=False):
        """Pop from the buffer if a new packet exists. If not just return None

        :param block: If true, blocks until a new message is received
        :return: proto if exists, else None
        :rtype: proto or None

        """
        if block:
            return self.proto_buffer.get()

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
            logger.warning("buffer overrun for {}".format(self.unix_path))

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
    def __init__(self, handle_callback, proto_class, base64_encoded, *args, **keys):
        self.handle_callback = handle_callback
        self.proto_class = proto_class
        self.base64_encoded = base64_encoded
        super().__init__(*args, **keys)

    def handle(self):
        """TODO"""
        if not self.base64_encoded:
            self.handle_proto()
        else:
            self.handle_log_visualize()

    def handle_proto(self):
        """If a specific protobuf class is passed in, this handler is called.

        It deserializes the incoming msg into the class and triggers the 
        handle callback.

        """
        if self.proto_class:
            self.handle_callback(self.proto_class.FromString(self.request[0]))
        else:
            raise Exception("proto_class is None but handle_proto called")

    def handle_log_visualize(self):
        """We send protobufs from our C++ code to python for visualization.
        If we used the handle_proto handler and passed in a proto_class, we
        would need to setup a sender/receiver pair for every protobuf we want
        to visualize. 
        
        So instead, we special case the communication coming from the ProtobufSink
        (C++ side). The ProtobufSink sends the typename prefixed at the beginning
        of the payload delimited by the TYPE_DELIMITER (!!!).

                              |         -- data --            |
        PackageName.TypeName!!!eW91Zm91bmR0aGVzZWNyZXRtZXNzYWdl

        This allows us to call LOG(VISUALIZE) _anywhere_ in C++ and 
        receive/decode here with minimum boilerplate code.

        """
        payload = self.request[0]
        result = base64.b64decode(payload)
        msg = self.proto_class()

        any_msg = Any.FromString(result)
        any_msg.Unpack(msg)
        self.handle_callback(msg)


def handler_factory(handle_callback, proto_class, base64_encoded):
    """To pass in an arbitrary handle callback into the SocketServer,
    we need to create a constructor that can create a Session object with
    appropriate handle function.

    :param handle_callback: The callback to run
    :param proto_class: The protobuf to unpack from (None if its encoded in the payload)
    :param base64_encoded: TODO

    """

    def create_handler(*args, **keys):
        return Session(handle_callback, proto_class, base64_encoded, *args, **keys)

    return create_handler
