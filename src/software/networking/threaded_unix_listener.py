import base64
import glob
import importlib
import inspect
import os
import queue
import socketserver
from threading import Thread

import proto
from google.protobuf.any_pb2 import Any


class ThreadedUnixListener:
    def __init__(self, unix_path, proto_class=None, max_buffer_size=3):

        """Receive protobuf over unix sockets and buffers them

        :param unix_path: The unix path to receive the new protobuf to plot
        :param proto_class: The protobuf to unpack from (None if its encoded in the payload)
        :param max_buffer_size: The size of the buffer

        """

        # cleanup the old path if it exists
        try:
            os.remove(unix_path)
        except:
            pass

        self.server = socketserver.UnixDatagramServer(
            unix_path, handler_factory(self.__buffer_protobuf, proto_class)
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
            print("buffer overrun for {}".format(self.unix_path))

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
    def __init__(self, handle_callback, proto_class=None, *args, **keys):
        self.handle_callback = handle_callback
        self.proto_class = proto_class
        super().__init__(*args, **keys)

    def handle(self):
        """Handle the two cases:

        1. Given the proto_class, decode the incoming data and trigger a callback.
           This is mostly used for direct protobuf communication.
        2. LOG(VISUALIZE) calls from g3log in the C++ code sending over stuff to
           visualize follows a specific format (see handle_log_visualize) we
           need to decode.

        """
        if self.proto_class:
            self.handle_proto()
        else:
            self.handle_log_visualize()

    def handle_proto(self):
        """If a specific protobuf class is passed in, this handler is called

        :param function: TODO
        :returns: TODO

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
        
        So instead, we special case the communication coming from the ProtobufSink,
        and send the typename prefixed at the beginning of the payload delimited
        by the TYPE_DELIMITER (!!!).

                              |         -- data --            |
        PackageName.TypeName!!!eW91Zm91bmR0aGVzZWNyZXRtZXNzYWdl

        This allows us to call LOG(VISUALIZE) _anywhere_ in C++ and 
        receive/decode here with minimum boilerplate code.

        """
        payload = self.request[0]
        type_name = str(payload.split(b"!!!")[0], "utf-8")
        proto_type = self.find_proto_class(type_name.split(".")[1])
        msg = proto_type()

        payload = base64.b64decode(payload.split(b"!!!")[1])

        any_msg = Any.FromString(payload)
        any_msg.Unpack(msg)

        self.handle_callback(msg)

    def find_proto_class(self, proto_class_name):
        """Search through all protobufs and return class of proto_type

        :param proto_class_name: String of the proto class name to search for

        """
        proto_path = os.path.dirname(proto.__file__)

        for file in glob.glob(proto_path + "**/*.py"):
            name = os.path.splitext(os.path.basename(file))[0]

            # Ignore __ files
            if name.startswith("__"):
                continue
            module = importlib.import_module("proto." + name)

            for member in dir(module):
                handler_class = getattr(module, member)
                if handler_class and inspect.isclass(handler_class):
                    if str(member) == proto_class_name:
                        return handler_class


def handler_factory(handle_callback, proto_class):
    """To pass in an arbitrary handle callback into the SocketServer,
    we need to create a constructor that can create a Session object with
    appropriate handle function.

    :param handle_callback: The callback to run
    :param proto_class: The protobuf to unpack from (None if its encoded in the payload)
    """

    def create_handler(*args, **keys):
        return Session(handle_callback, proto_class, *args, **keys)

    return create_handler
