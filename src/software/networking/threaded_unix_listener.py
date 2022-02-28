import socketserver
import time
import base64
import os
from webbrowser import get
from google.protobuf import text_format
from google.protobuf.any_pb2 import Any
from threading import Thread
import proto
import queue
import glob
import importlib
import inspect
import os


class ThreadedUnixListener:
    def __init__(self, unix_path, max_buffer_size=3, convert_from_any=True):

        """Receive protobuf over unix sockets and buffers them

        :param unix_path: The unix path to receive the new protobuf to plot
        :param max_buffer_size: The size of the buffer
        :param convert_from_any: Convert from any

        """

        # cleanup the old path if it exists
        try:
            os.remove(unix_path)
        except:
            pass

        self.server = socketserver.UnixDatagramServer(
            unix_path, handler_factory(self.__buffer_protobuf, convert_from_any)
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
    def __init__(self, handle_callback, convert_from_any, *args, **keys):
        self.handle_callback = handle_callback
        self.convert_from_any = convert_from_any
        super().__init__(*args, **keys)

    def handle(self):
        """Decode the base64 request and unpack from Any if we are receiving
        an Any protobuf. If not, just unpack directly into the type provided.

        Then, trigger the handle callback

        """
        p = self.request[0]
        type_name = str(p.split(b"!!!")[0], "utf-8")
        proto_type = self.find_proto_class(type_name.split(".")[1])
        msg = proto_type()
        p = base64.b64decode(p.split(b"!!!")[1])

        if self.convert_from_any:
            any_msg = Any.FromString(p)
            any_msg.Unpack(msg)
        else:
            msg = proto_type.FromString(p)

        self.handle_callback(msg)

    """Search through all protobufs and return class of proto_type

    param: proto_class_name: String of the proto class name to search for
    """
    def find_proto_class(self, proto_class_name):
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


def handler_factory(handle_callback, convert_from_any):
    """To pass in an arbitrary handle callback into the SocketServer,
    we need to create a constructor that can create a Session object with
    appropriate handle function.

    :param handle_callback: The callback to run
    :param convert_from_any: If true, the message needs to be decoded
                             into Any before into the proto_type
    """

    def create_handler(*args, **keys):
        return Session(handle_callback, convert_from_any, *args, **keys)

    return create_handler
