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

        def start(self):
            while True:
                proto = self.proto_receiver.buffer.get()
                if proto.DESCRIPTOR.full_name in self.proto_map:
                    for buffer in self.proto_map[proto.DESCRIPTOR.full_name]:
                        try:
                            buffer.put_nowait(proto)
                        except queue.Full:
                            pass

        def registerObserver(self, proto_type, buffer):
            if proto_type in self.proto_map:
                self.proto_map[proto_type.DESCRIPTOR.full_name].append(buffer)
            else:
                self.proto_map[proto_type.DESCRIPTOR.full_name] = [buffer]