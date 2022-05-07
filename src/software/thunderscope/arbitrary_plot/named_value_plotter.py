import random
import time
from collections import deque

import pyqtgraph as pg
from proto.visualization_pb2 import NamedValue
from pyqtgraph.Qt import QtGui

from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

DEQUE_SIZE = 1000
MIN_Y_RANGE = 0
MAX_Y_RANGE = 100
TIME_WINDOW_TO_DISPLAY_S = 20


class NamedValuePlotter(object):

    """ Plot named values in real time with a scrolling plot """

    def __init__(self, buffer_size=1000):
        """Initializes NamedValuePlotter.

        :param buffer_size: The size of the buffer to use for plotting.

        """
        self.win = pg.plot()
        self.plots = {}
        self.data_x = {}
        self.data_y = {}
        self.legend = pg.LegendItem((80, 60), offset=(70, 20))
        self.legend.setParentItem(self.win.graphicsItem())
        self.time = time.time()
        self.named_value_buffer = ThreadSafeBuffer(buffer_size, NamedValue)

    def refresh(self):
        """Refreshes NamedValuePlotter and updates data in the respective
        plots.

        """

        # Dump the entire buffer into a deque. This operation is fast because
        # its just consuming data from the buffer and appending it to a deque.
        for _ in range(self.named_value_buffer.queue.qsize()):
            named_value = self.named_value_buffer.get(block=False)

            # If named_value is new, create a plot and for the new value and
            # add it to necessary maps
            if named_value.name not in self.plots:
                self.plots[named_value.name] = self.win.plot(
                    pen=QtGui.QColor(
                        random.randint(100, 255),
                        random.randint(100, 255),
                        random.randint(100, 255),
                    ),
                    name=named_value.name,
                    disableAutoRange=True,
                    brush=None,
                )

                self.plots[named_value.name].setDownsampling(method="peak")
                self.data_x[named_value.name] = deque([], DEQUE_SIZE)
                self.data_y[named_value.name] = deque([], DEQUE_SIZE)
                self.legend.addItem(self.plots[named_value.name], named_value.name)

            # Add incoming data to existing deques of data
            self.data_x[named_value.name].append(time.time() - self.time)
            self.data_y[named_value.name].append(named_value.value)

            # Update the data
            self.plots[named_value.name].setData(
                self.data_x[named_value.name], self.data_y[named_value.name]
            )

        self.win.setRange(
            xRange=[
                time.time() - self.time - TIME_WINDOW_TO_DISPLAY_S,
                time.time() - self.time,
            ],
        )
