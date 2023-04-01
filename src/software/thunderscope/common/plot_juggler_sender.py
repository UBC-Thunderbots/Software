import time
import json
from threading import Thread
from google.protobuf.json_format import MessageToJson, MessageToDict

from proto.visualization_pb2 import PlotJugglerValue
from software.py_constants import PLOTJUGGLER_GUI_DEFAULT_HOST, PLOTJUGGLER_GUI_DEFAULT_PORT
from software.python_bindings import ThreadedUdpSender
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class PlotJugglerSender:
    """Forwards plotjuggler values to plotjuggler over UDP"""

    def __init__(self, buffer_size=20):
        self.log_buffer = ThreadSafeBuffer(buffer_size, PlotJugglerValue)

        self.udp_sender = ThreadedUdpSender(PLOTJUGGLER_GUI_DEFAULT_HOST, PLOTJUGGLER_GUI_DEFAULT_PORT, False)

        # We want to set daemon to true so that the program can exit even
        # if the thread is still running
        self.thread = Thread(target=self.start, daemon=True)
        self.thread.start()

    def start(self):
        i = 0
        while True:
            # TODO: Can we label blue, yellow, and sim differently?!
            # TODO: Similar to proto plotter, parse protos. Might help with labelling
            # TODO: Receive packets from robot as well
            plot_juggler_proto = self.log_buffer.get(block=True)
            plot_juggler_json = MessageToDict(plot_juggler_proto, preserving_proto_field_name=True)
            # if i % 2 == 0:
            #     plot_juggler_json['blue'] = plot_juggler_json.pop('data')
            # else:
            #     plot_juggler_json['yellow'] = plot_juggler_json.pop('data')

            # i += 1
            # plot_juggler_json["blue"] = {'time': time.time()}
            self.udp_sender.sendString(json.dumps(plot_juggler_json))
