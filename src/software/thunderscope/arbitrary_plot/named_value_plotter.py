import pyqtgraph as pg
import time
from software.networking.threaded_unix_listener import ThreadedUnixListener
import software.thunderscope.constants as constants
from proto.visualization_pb2 import NamedValue
import queue
import random
from pyqtgraph.Qt import QtGui
from collections import deque
DEQUE_SIZE = 500
MIN_Y_RANGE = 0
MAX_Y_RANGE = 100
TIME_WINDOW_TO_DISPLAY_S = 10

class NamedValuePlotter(object):
    def __init__(self, buffer_size=10):

        self.win = pg.plot()
        self.plots = {}
        self.data_x = {}
        self.data_y = {}
        self.legend = pg.LegendItem((80, 60), offset=(70, 20))
        self.legend.setParentItem(self.win.graphicsItem())
        self.time = time.time()
        self.named_value_buffer = queue.Queue(buffer_size)

    @property
    def plot(self):
        return self.win

    """Refreshes NamedValuePlotter and updates data in "plots" with data from "named_value_buffer"
    """
    def refresh(self):
        try:
            named_value = self.named_value_buffer.get_nowait()

            #if named_value is new, create a plot and for the new value and add it to necessary maps
            if named_value.name not in self.plots:
                self.plots[named_value.name] = self.win.plot(
                    pen=QtGui.QColor(
                        random.randint(0, 255),
                        random.randint(0, 255),
                        random.randint(0, 255),
                    ),
                    name=named_value.name,
                    brush=None,
                )
                self.plots[named_value.name].setDownsampling(method="peak")
                self.data_x[named_value.name] = deque([], DEQUE_SIZE)
                self.data_y[named_value.name] = deque([], DEQUE_SIZE)
                self.legend.addItem(self.plots[named_value.name], named_value.name)

            #Add incoming data to existing deques of data
            self.data_x[named_value.name].append(time.time() - self.time)
            self.data_y[named_value.name].append(named_value.value)
            self.plots[named_value.name].setData(
                self.data_x[named_value.name], self.data_y[named_value.name]
            )
            self.win.setRange(
                yRange=[MIN_Y_RANGE, MAX_Y_RANGE],
                xRange=[time.time() - self.time - TIME_WINDOW_TO_DISPLAY_S, time.time() - self.time],
                disableAutoRange=True,
            )
        except queue.Empty as empty:
            return
