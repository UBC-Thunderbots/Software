import json
import socket
import time

from software.py_constants import PLOTJUGGLER_GUI_DEFAULT_HOST, PLOTJUGGLER_GUI_DEFAULT_PORT


class PlotJugglerSender:
    """Sends named values to PlotJuggler over UDP, in the same JSON format as the
    C++ PlotJugglerSink (a serialized TbotsProto::PlotJugglerValue)."""

    def __init__(self, interface=None):
        """
        :param interface: the network interface to send over, or None for the default
        """
        self.address = (PLOTJUGGLER_GUI_DEFAULT_HOST, PLOTJUGGLER_GUI_DEFAULT_PORT)
        self.socket = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
        if interface:
            self.socket.setsockopt(
                socket.IPPROTO_IPV6,
                socket.IPV6_MULTICAST_IF,
                socket.if_nametoindex(interface),
            )

    def send(self, data):
        """Send a set of named values stamped with the current time

        :param data: dict of series name to value
        """
        message = json.dumps({"timestamp": time.time(), "data": data})
        try:
            self.socket.sendto(message.encode(), self.address)
        except OSError:
            # PlotJuggler is optional, never fail the test over it
            pass

    def close(self):
        self.socket.close()
