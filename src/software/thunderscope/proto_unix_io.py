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

    - Classes can also call send_proto to send data to any registered observers

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
        self.all_proto_observers = []
        self.unix_senders = {}
        self.unix_listeners = {}
        self.send_proto_to_observer_threads = {}
        self.stop_running = False

    def __send_proto_to_observers(self, receive_buffer: ThreadSafeBuffer):
        """Given a ThreadSafeBuffer (receive_buffer) consume it and
        send place it in the other buffers.

        :param receive_buffer: The queue to consume from

        """
        while not self.stop_running:
            proto = receive_buffer.get()

            if proto.DESCRIPTOR.full_name in self.proto_observers:
                for buffer in self.proto_observers[proto.DESCRIPTOR.full_name]:
                    try:
                        buffer.put(proto, block=False)
                    except queue.Full:
                        pass

            for buffer in self.all_proto_observers:
                try:
                    buffer.put(proto, block=False)
                except queue.Full:
                    print("Buffer registered to receive everything dropped data")
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

    def register_to_observe_everything(self, buffer):
        """Register a buffer to observe all incoming protobufs

        :param buffer: buffer to push protos onto
        
        """
        self.all_proto_observers.append(buffer)

    def send_proto(self, proto_class, data, block=False, timeout=None):
        """Send the data to all register_observers

        :param proto_class: The class to send
        :param data: The data to send
        :param block: If block is True, then block until a free space opens up
                      to put the proto. Otherwise, proto will be dropped if queue is full.
        :param timeout: If block is True, then wait for this many seconds

        """
        if proto_class.DESCRIPTOR.full_name in self.proto_observers:
            for buffer in self.proto_observers[proto_class.DESCRIPTOR.full_name]:
                buffer.put(data, block, timeout)

        for buffer in self.all_proto_observers:
            try:
                buffer.put(data, block, timeout)
            except queue.Full:
                # print("Buffer registered to receive everything dropped data")
                pass

    def attach_unix_sender(self, runtime_dir, unix_path, proto_class):
        """Creates a unix sender and registers an observer
        of the proto_class to send the data over the unix_path socket.
        
        :param runtime_dir: The runtime_dir where all protos will be sent to
        :param unix_path: The unix socket path within the runtime_dir to open
        :param proto_class: The protobuf type to send

        """
        sender = ThreadedUnixSender(
            unix_path=runtime_dir + unix_path, proto_type=proto_class
        )
        self.unix_senders[proto_class.DESCRIPTOR.full_name] = sender
        self.register_observer(proto_class, sender.proto_buffer)

    def attach_unix_receiver(
        self, runtime_dir, unix_path="", proto_class=None, from_log_visualize=False
    ):
        """Creates a unix listener of that protobuf type and provides
        incoming data to registered observers.
        
        NOTE: We have to special case log visualize calls because the unix path
        is autogenerated from the descriptor name. The data is also base64 encoded
        because of g3log treating \n characters as a new log msg.

        :param runtime_dir: The runtime_dir where all protos will be sent to
        :param unix_path: The unix path within the runtime_dir to send data over
        :param proto_class: The prototype to send
        :param from_log_visualize: If the protobuf is coming from LOG(VISUALIZE)

        """
        listener = ThreadedUnixListener(
            runtime_dir + f"/{proto_class.DESCRIPTOR.full_name}"
            if from_log_visualize and not unix_path
            else runtime_dir + unix_path,
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

    def force_close(self):
        self.stop_running = True
        for sender in self.unix_senders.items():
            sender[1].force_stop()
        for listener in self.unix_listeners.items():
            listener[1].force_stop()
