import socketserver
from struct import unpack
from os import unlink
from proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket

class StoppableServer:
    stop = False

    def __init__(self, server):
        self.server = server

    def serve_till_stopped(self):
        while not self.stop:
            self.server.handle_request()

    def force_stop(self):
        self.stop = True
        self.server.server_close()

    def start(self):
        self.stop = False
        self.serve_till_stopped()


class Session(socketserver.BaseRequestHandler):
    def handle(self):
        header = self.request.recv(4)
        message_length, = unpack('>I', header)  # unpack always returns a tuple.

        message = self.request.recv(message_length)
        protobuf_message = SSL_WrapperPacket.SSL_WrapperPacket
        print("Message: " + protobuf_message.ParseFromString(message))

def main():
    address = './socket'

    try:
        unlink(address)
    except OSError as e:
        print(e)
        pass

    receiver = StoppableServer(socketserver.UnixStreamServer(address, Session))
    receiver.start()

if __name__ == '__main__':
    main()
