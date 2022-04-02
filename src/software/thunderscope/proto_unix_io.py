import queue
from threading import Thread

import software.thunderscope.constants as constants
from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.networking.threaded_unix_sender import ThreadedUnixSender


class ProtoUnixIO:

    """The ProtoUnixIO is responsible for communicating protobufs over unix sockets.


                                                          Register Protobuf
                                                          to a buffer to
                                                          receive data
                                                                 │
                                                                 │
                                                                 ▼
        ProtoUnixReceiver ┌───────────────────┐           ┌──┬──┬──┬──┬──┐
                ──────────>                   ├──────────>│  │  │  │  │  │
                          │                   │           └──┴──┴──┴──┴──┘
                          │    ProtoUnixIO    │                Queue 
        ProtoUnixSender   │                   │
                <─────────┤                   <──────────── send_proto
                          └───────────────────┘

    Classes can register as an observer by providing a protobuf type to observe
    and a queue to place incoming data. We pass in a queue rather than a callback
    so not bog down the ProtoUnixIO class and allowing the observers to process
    the data at their own pace.

    Clases can send proto and ProtoUnixIO will send that data to the registered
    observers.

    attach_unix_receiver can be called with a protobuf type and unix socket path
    to receive data over the unix socket to get sent to all the registered observers

    attach_unix_sender can be called with a protobuf type and unix socket path
    to send data over the socket, if send_proto was called.

    TL;DR This class manages all IO over unix sockets and provides a send proto
    function and a register_observer interface to reiceve data over unix sockets.

    """

    def __init__(self):
        # Mapping from ProtoType.DESCRIPTOR.full_name -> buffer
        self.proto_observers = {}
        self.unix_senders = {}
        self.unix_listeners = {}
        self.send_proto_to_observer_threads = {}

    def __send_proto_to_observers(self, receive_buffer):
        """Given a queue (receive_buffer) consume it and
        send place it in the other buffers.

        :param receive_buffer: The queue to consume from

        """
        while True:
            proto = receive_buffer.get()
            if proto.DESCRIPTOR.full_name in self.proto_observers:
                for buffer in self.proto_observers[proto.DESCRIPTOR.full_name]:
                    try:
                        buffer.put_nowait(proto)
                    except queue.Full:
                        pass

    def register_observer(self, proto_class, buffer):
        """Register a widget to consume from a given protobuf class

        :param proto_class: Class of protobuf to consume
        :param buffer: buffer from the widget to register

        """
        if proto_class.DESCRIPTOR.full_name in self.proto_observers:
            self.proto_observers[proto_class.DESCRIPTOR.full_name].append(buffer)
        else:
            self.proto_observers[proto_class.DESCRIPTOR.full_name] = [buffer]

    def send_proto(self, proto_class, data):
        """Send the data to all register_observers

        :param proto_class: The class to send
        :param data: The data to send

        """
        for buffer in self.proto_observers[proto_class.DESCRIPTOR.full_name]:
            try:
                buffer.put_nowait(data)
            except queue.Full:
                pass

    def attach_unix_sender(self, unix_path, proto_class):
        """Creates a unix sender and registers an observer
        of the proto_class to send the data over the unix_path socket.
        
        :param unix_path: The unix socket path to open
        :param proto_class: The protobuf type to send

        """
        sender = ThreadedUnixSender(unix_path=unix_path)
        self.unix_senders[proto_class.DESCRIPTOR.full_name] = sender
        self.register_observer(proto_class, sender.proto_buffer)

    def attach_unix_receiver(
        self, unix_path, proto_class=None, from_log_visualize=False
    ):
        """Creates a unix listener of that protobuf type and provides
        incoming data to registered observers.
        
        NOTE: We have to special case log visualize calls

        :param unix_path: The unix path to send data over
        :param proto_class: The prototype to send

        """
        listener = ThreadedUnixListener(
            unix_path, proto_class=proto_class, base64_encoded=from_log_visualize
        )
        key = proto_class.DESCRIPTOR.full_name
        self.unix_listeners[key] = listener
        self.send_proto_to_observer_threads[key] = Thread(
            target=self.__send_proto_to_observers, args=(listener.proto_buffer,)
        )
        self.send_proto_to_observer_threads[key].start()
