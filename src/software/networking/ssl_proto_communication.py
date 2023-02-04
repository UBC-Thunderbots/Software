import socket
import google.protobuf.internal.encoder as encoder

class SslSocket(object):
    def __init__(self, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect(("", port))

    def send(self, proto):
        size = proto.ByteSize()

        self.socket.send(
            encoder._VarintBytes(size) + proto.SerializeToString()
        )

    def receive(self, proto_type) :
        try : 
            proto_class = eval(proto_type)
            print("actually works?")
        except NameError:
            raise TypeError(f"Unknown proto type: '{proto_type}'")


