import socket
import google.protobuf.internal.encoder as encoder
import google.protobuf.internal.decoder as decoder

class SslSocket(object):
    RECEIVE_BUFFER_SIZE = 9000

    def __init__(self, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect(("", port))

    def send(self, proto):
        size = proto.ByteSize()

        self.socket.send(
            encoder._VarintBytes(size) + proto.SerializeToString()
        )

    def receive(self, proto_type) :
        responses = list()

        try : 
            proto = proto_type()
        except NameError:
            raise TypeError(f"Unknown proto type: '{proto_type}'")

        response_data = self.socket.recv(
                SslSocket.RECEIVE_BUFFER_SIZE
        )

        offset = 0
        while offset < len(response_data):
            msg_len, new_pos = decoder._DecodeVarint32(response_data, offset)
            offset = new_pos
            ci_output = proto_type()
            if (offset + msg_len > len(response_data)):
                response_data += self.socket.recv(
                        offset + msg_len - len(response_data)
                )
            ci_output.ParseFromString(response_data[offset : offset + msg_len])
            offset += msg_len
            responses.append(ci_output)

        return responses

