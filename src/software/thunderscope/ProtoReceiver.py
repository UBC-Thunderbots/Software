import software.thunderscope.constants as constants
import queue
from threading import Thread
from software.networking.threaded_unix_listener import ThreadedUnixListener
import software.thunderscope.constants as constants

class ProtoReceiver:
    def __init__(self):
        self.proto_map = dict()
        self.proto_receiver = ThreadedUnixListener(
            constants.UNIX_SOCKET_BASE_PATH + "protobuf",
            convert_from_any=True,
            max_buffer_size=3,
        )
        self.thread = Thread(target=self.start)
        self.thread.start()

    """
    Distributes protobuf from the proto_receiver to all widgets that consume that specific protobuf
    """
    def start(self):
        while True:
            proto = self.proto_receiver.buffer.get()
            if proto.DESCRIPTOR.full_name in self.proto_map:
                for buffer in self.proto_map[proto.DESCRIPTOR.full_name]:
                    try:
                        buffer.put_nowait(proto)
                    except queue.Full:
                        pass
    
    """Register a widget to consume from a given protobuf class

    param: proto_type: Class of protobuf to consume
    param: buffer: buffer from the widget to register
    """
    def register_observer(self, proto_type, buffer):
        if proto_type in self.proto_map:
            self.proto_map[proto_type.DESCRIPTOR.full_name].append(buffer)
        else:
            self.proto_map[proto_type.DESCRIPTOR.full_name] = [buffer]
