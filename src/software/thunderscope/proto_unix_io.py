from threading import Thread
import queue

from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.networking.threaded_unix_sender import ThreadedUnixSender
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


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
                          │    ProtoUnixIO    │           ThreadSafeBuffer 
        ProtoUnixSender   │                   │
                <─────────┤                   <──────────── send_proto
                          └───────────────────┘

    Observing Protobuf:

    - Classes can register as an observer by providing a protobuf type to
      observe and a ThreadSafeBuffer to place incoming data. We pass in a
      ThreadSafeBuffer rather than a callback to not bog down this class.
      We don't want slow callbacks from a single observer to hold up other
      observers from receiving new data.

    Sending Protobuf:

    - Clases can also call send_proto to send data to any registered observers

    Unix IO:

    - attach_unix_receiver() configures a unix receiver to receive protobufs
      over a unix socket and send data to all observers of that proto type.

    - attach_unix_sender() configures a unix sender (it is an observer as well)
      and relays data from send_proto over the socket.

    TL;DR This class manages inter-thread communication through register_observer
    and send_proto calls. If unix senders/receivers are attached to a proto type,
    then the data is also sent/received over the sockets.

    """

    def __init__(self):
        # Mapping from ProtoType.DESCRIPTOR.full_name -> buffer
        self.proto_observers = {}
        self.unix_senders = {}
        self.unix_listeners = {}
        self.send_proto_to_observer_threads = {}

    def __send_proto_to_observers(self, receive_buffer: ThreadSafeBuffer):
        """Given a ThreadSafeBuffer (receive_buffer) consume it and
        send place it in the other buffers.

        :param receive_buffer: The queue to consume from

        """
        while True:
            proto = receive_buffer.get()

            if proto.DESCRIPTOR.full_name in self.proto_observers:
                for buffer in self.proto_observers[proto.DESCRIPTOR.full_name]:
                    try:
                        buffer.put(proto, block=False)
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
        if proto_class.DESCRIPTOR.full_name not in self.proto_observers:
            raise KeyError(
                "No observers registered for {}!".format(
                    proto_class.DESCRIPTOR.full_name
                )
            )

        for buffer in self.proto_observers[proto_class.DESCRIPTOR.full_name]:
            buffer.put(data)

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
        self, unix_path, proto_class=None, from_log_visualize=False,
    ):
        """Creates a unix listener of that protobuf type and provides
        incoming data to registered observers.
        
        NOTE: We have to special case log visualize calls because the unix path
        is autogenerated from the descriptor name. The data is also base64 encoded
        because of g3log treating \n characters as a new log msg.

        :param unix_path: The unix path to send data over
        :param proto_class: The prototype to send
        :param log_visualize: If the protobuf is coming from LOG(VISUALIZE)

        """
        listener = ThreadedUnixListener(
            unix_path + "/{}".format(proto_class.DESCRIPTOR.full_name)
            if from_log_visualize
            else unix_path,
            proto_class=proto_class,
            is_base64_encoded=from_log_visualize,
        )
        key = proto_class.DESCRIPTOR.full_name
        self.unix_listeners[key] = listener
        self.send_proto_to_observer_threads[key] = Thread(
            target=self.__send_proto_to_observers,
            args=(listener.proto_buffer,),
            daemon=True,
        )
        self.send_proto_to_observer_threads[key].start()
