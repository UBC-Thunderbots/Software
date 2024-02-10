import random
import time
from collections import deque

import pyqtgraph as pg
from proto.visualization_pb2 import NamedValue
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt import QtGui, QtCore

from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class ProtoPlotter(QWidget):

    """Plot the protobuf data in a pyqtgraph plot

    In-order to make the plotter as flexible as possible, we need dependency
    inject a way to extract data from the incoming protobufs. This is so that
    the user can control the way the data is plotted.

    Examples:

    NamedValueProto: we can plot the value directly
    RobotStatus: the data comes in over multiple packets, with each one having
    a different robot id. We need to append the robot id to the name so that
    the values are plotted separated by the ID. We also might want to plot
    different parts of the robot status in different plots.

    So we define a configuration dictionary that looks like this:

    {
        protobuf_type_1: data_extractor_function_1,
        protobuf_type_2: data_extractor_function_2,
        ...
    }

    The plotter will observe the provided protobuf types and call the associated
    data extractor function to extract the data.

    The data_extractor_function should take a protobuf return a dictionary of
    the form:

    {
        "name_1": value_1,
        "name_2": value_2,
        ...
    }

    """

    def __init__(
        self,
        min_y,
        max_y,
        window_secs,
        configuration,
        plot_rate_hz=60,
        buffer_size=1000,
    ):
        """Initializes NamedValuePlotter.

        :param min_y: Minimum y value to display
        :param max_y: Maximum y value to display
        :param window_secs: How many seconds to show in the x axis
        :param configuration: A dictionary of protobuf types to data extractor
        :param plot_rate_hz: How many times per second to update the plot
        :param buffer_size: The size of the buffer to use for plotting.

        """
        QWidget.__init__(self)

        self.win = pg.PlotWidget()
        self.win.disableAutoRange()
        self.win.setYRange(min_y, max_y)

        self.data = {}
        self.plots = {}
        self.data_x = {}
        self.data_y = {}
        self.legend = pg.LegendItem((80, 60), offset=(70, 20))
        self.legend.setParentItem(self.win.graphicsItem())
        self.window_secs = window_secs
        self.configuration = configuration

        self.buffers = {
            key: ThreadSafeBuffer(buffer_size, key) for key in configuration.keys()
        }

        self.time = time.time()
        self.last_update_time = time.time()
        self.last_incoming_value = {}
        self.update_interval = 1.0 / plot_rate_hz
        self.buffer_size = buffer_size

    def isVisible(self):
        return self.win.isVisible()

    def refresh(self):
        """Refreshes ProtoPlotter and updates data in the respective
        plots.

        """

        # Dump the entire buffer into a deque. This operation is fast because
        # its just consuming data from the buffer and appending it to a deque.
        for proto_class, buffer in self.buffers.items():
            for _ in range(buffer.queue.qsize()):

                data = self.configuration[proto_class](buffer.get(block=False))

                # If named_value is new, create a plot and for the new value and
                # add it to necessary maps
                for name, value in data.items():
                    if name not in self.plots:
                        self.data_x[name] = deque([], self.buffer_size)
                        self.data_y[name] = deque([], self.buffer_size)
                        self.plots[name] = self.win.plot(
                            pen=QtGui.QColor(
                                random.randint(100, 255),
                                random.randint(100, 255),
                                random.randint(100, 255),
                            ),
                            name=name,
                            disableAutoRange=True,
                            brush=None,
                        )

                        self.plots[name].setDownsampling(method="peak")
                        self.legend.addItem(self.plots[name], name)

                    # Add incoming data to existing deques of data
                    self.data_x[name].append(time.time() - self.time)
                    self.data_y[name].append(value)

        if self.last_update_time + self.update_interval > time.time():
            return

        self.last_update_time = time.time()

        for named_value, plot in self.plots.items():
            # Update the data
            plot.setData(self.data_x[named_value], self.data_y[named_value])

        self.win.setRange(
            xRange=[
                time.time() - self.time - self.window_secs,
                time.time() - self.time,
            ],
        )
