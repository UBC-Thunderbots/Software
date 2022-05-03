import random
import time
from collections import deque
import numpy

import pyqtgraph as pg
from proto.visualization_pb2 import NamedValue
from pyqtgraph.Qt import QtGui

from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

DEQUE_SIZE = 100000
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
        self.total_time = 0
        self.times_visualized = 0
        self.win.disableAutoRange()

    def refresh(self):
        """Refreshes NamedValuePlotter and updates data in the respective
        plots.

        """
        self.times_visualized += 1
        start_time = time.time()

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
                self.data_x[named_value.name] = numpy.zeros(DEQUE_SIZE)
                self.data_y[named_value.name] = numpy.zeros(DEQUE_SIZE)
                self.legend.addItem(self.plots[named_value.name], named_value.name)

            # Add incoming data to existing deques of data
            self.data_x[named_value.name][0:-1] = self.data_x[named_value.name][1:]
            self.data_x[named_value.name][-1] = (time.time() - self.time)

            self.data_y[named_value.name][0:-1] = self.data_y[named_value.name][1:]
            self.data_y[named_value.name][-1] = named_value.value

        t1 = time.time()
        for named_value, plot in self.plots.items():
            # Update the data
            plot.setData(
                self.data_x[named_value], self.data_y[named_value]
            )

        t2 = time.time()
        print("Plotting time: {}".format(t2 - t1))

        t1 = time.time()
        self.win.setRange(
            xRange=[
                time.time() - self.time - TIME_WINDOW_TO_DISPLAY_S,
                time.time() - self.time,
            ],
        )
        # self.win.autoRange()
        t2 = time.time()
        print("Setting range time: {}".format(t2 - t1))

        end_time = time.time()
        self.total_time += end_time - start_time
        print("Avg Refresh time: {}".format(self.total_time / self.times_visualized))
        print("================")

# 0.045 No disable autorange, and no autorange, and no setrange
# 0.016    disable autorange, and no autorange, and no setrange
# 0.020    disable autorange, and no autorange, and    setrange
# 0.029    disable autorange, and    autorange, and    setrange
# 0.039 No disable autorange, and no autorange, and    setrange

# Can auto range, it will automatically get updated by new data, but will also show all data instead of a certain window