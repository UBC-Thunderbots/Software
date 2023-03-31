import time
import json
from threading import Thread
from google.protobuf.json_format import MessageToJson

from proto.visualization_pb2 import PlotJugglerValue
from software.py_constants import PLOTJUGGLER_DEFAULT_HOST, PLOTJUGGLER_DEFAULT_PORT
from software.python_bindings import ThreadedUdpSender
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer



class PlotJugglerSender:
    """Forwards plotjuggler values to plotjuggler over UDP"""

    def __init__(self, buffer_size=20):
        self.log_buffer = ThreadSafeBuffer(buffer_size, PlotJugglerValue)

        self.udp_sender = ThreadedUdpSender(PLOTJUGGLER_DEFAULT_HOST, PLOTJUGGLER_DEFAULT_PORT, False)

        # We want to set daemon to true so that the program can exit even
        # if the thread is still running
        self.thread = Thread(target=self.start, daemon=True)
        self.thread.start()

    def start(self):
        while True:
            # TODO: Can we label blue, yellow, and sim differently?!
            # TODO: Similar to proto plotter, parse protos. Might help with labelling
            # TODO: Receive packets from robot as well
            plot_juggler_proto = self.log_buffer.get(block=True)
            plot_juggler_json = MessageToJson(plot_juggler_proto, preserving_proto_field_name=True)
            self.udp_sender.sendString(plot_juggler_json)
            time.sleep(0.016)
