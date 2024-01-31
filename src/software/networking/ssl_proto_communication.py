from __future__ import annotations

import socket
import google.protobuf.internal.encoder as encoder
import google.protobuf.internal.decoder as decoder
import google.protobuf.message as protobuf_message


class SslSocketProtoParseException(Exception):
    """
    A custom exception raised by the SSL Socket class when a proto cannot be parsed
    """


class SslSocket(object):
    """
    The SSL Socket class is responsible for communication with SSL protos from SSL binaries. The encoding that SSL uses
    is slightly different from our encoding when we send protobufs between different processes (and robots).

    Each SSL Proto message is preceded by an uvarint containing the message size in bytes, so we must read a certain
    buffered amount, and then read the rest of the message as necessary.

    Encoding details can be seen here:
    https://github.com/RoboCup-SSL/ssl-game-controller/blob/master/cmd/ssl-auto-ref-client/README.md
    """

    RECEIVE_BUFFER_SIZE = 9000

    def __init__(self, port: int) -> None:
        """
        Open a TCP socket with the given port, to communicate with other processes. It binds the socket to INADDR_ANY
        which binds the socket to all local interfaces, meaning that it will listen to traffic on the specified port on
        ethernet, wifi,...

        :param port the port to bind to
        """
        # bind to all local interfaces, TCP
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect(("", port))

    def send(self, proto: protobuf_message.Message) -> None:
        """
        Send the proto through the socket.

        :param proto proto to send
        """
        # When streaming multiple Protobuf messages, we need to delimit distinct messages.  Protobufs are not
        # self-delimiting, so the common practice for sending delimited Protobuf messages is to write the size of the
        # packet in var-int form followed by the serialized message bytes.
        # https://cwiki.apache.org/confluence/display/GEODE/Delimiting+Protobuf+Messages
        size = proto.ByteSize()

        # Send a request to the host with the size of the message
        self.socket.send(encoder._VarintBytes(size) + proto.SerializeToString())

    def receive(
        self, proto_type: type[protobuf_message.Message]
    ) -> List[protobuf_message.Message]:
        """
        Receives proto(s) on the socket and returns them, given the proto type to expect. This function is blocking

        :param proto_type the proto type to parse received data as

        :return: a list of protos of the provided type that were received from the socket

        :raises TypeError:                      if the given proto_type isn't a known proto type
        :raises SslSocketProtoParseException:   if the received data from the socket isn't parseable as the given proto
        """
        responses = list()

        try:
            proto_type()
        except NameError:
            raise TypeError(f"Unknown proto type: '{proto_type}'")

        response_data = self.socket.recv(SslSocket.RECEIVE_BUFFER_SIZE)

        offset = 0
        while offset < len(response_data):
            msg_len, new_pos = decoder._DecodeVarint32(response_data, offset)
            offset = new_pos
            ci_output = proto_type()

            # we didn't receive enough data to parse this message, to get more
            if offset + msg_len > len(response_data):
                response_data += self.socket.recv(offset + msg_len - len(response_data))

            try:
                ci_output.ParseFromString(response_data[offset : offset + msg_len])
            except protobuf_message.DecodeError as err:
                raise SslSocketProtoParseException(
                    "Error parsing proto: {}".format(err)
                )

            if not ci_output.IsInitialized():
                raise SslSocketProtoParseException(
                    f"Improper proto of type '{proto_type}' parsed"
                )

            offset += msg_len
            responses.append(ci_output)

        return responses

    def close(self) -> None:
        """
        Closes the socket associated with this object.
        """
        self.socket.close()
