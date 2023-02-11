import socket
import google.protobuf.internal.encoder as encoder
import google.protobuf.internal.decoder as decoder

class SslSocketProtoParseException(Exception):
    pass

class SslSocket(object):
    RECEIVE_BUFFER_SIZE = 9000

    def __init__(self, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect(("", port))

    def send(self, proto):
        # https://cwiki.apache.org/confluence/display/GEODE/Delimiting+Protobuf+Messages
        size = proto.ByteSize()

        # Send a request to the host with the size of the message
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
            try :
                ci_output.ParseFromString(response_data[offset : offset + msg_len])
            except google.protobuf.message.DecodeError as err :
                raise SsslSocketProtoParseException("Error parsing proto: " + err.args)

            if not ci_output.IsInitialized():
                raise SslSocketProtoParseException("Improper proto of type '{proto_type}' parsed")
            offset += msg_len
            responses.append(ci_output)

        return responses

