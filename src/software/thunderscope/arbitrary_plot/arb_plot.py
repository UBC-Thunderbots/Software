import pyqtgraph as pg
import time
from software.networking.threaded_unix_listener import ThreadedUnixListener
import software.thunderscope.constants as constants
from proto.visualization_pb2 import NamedValue
import queue    
import random
from pyqtgraph.Qt import QtGui
from collections import deque

class NamedValuePlotter(object):
    def __init__(self):
        self.named_val_reciever = ThreadedUnixListener(
              constants.UNIX_SOCKET_BASE_PATH + NamedValue.DESCRIPTOR.full_name, NamedValue, convert_from_any=True)

        self.win = pg.plot()
        self.plots = {}
        self.data_x = {}
        self.data_y = {}
        self.legend = pg.LegendItem((80,60), offset=(70,20))
        self.legend.setParentItem(self.win.graphicsItem())
        self.time = time.time()

    @property
    def plot(self):
        return self.win

    def refresh(self):
        for x in range(0, 1):
            try:
                named_value = self.named_val_reciever.buffer.get_nowait()

                if named_value.name not in self.plots:
                    self.plots[named_value.name] =\
                            self.win.plot(
                                    pen=QtGui.QColor(
                                        random.randint(0,255),
                                        random.randint(0,255),
                                        random.randint(0,255)), name=named_value.name, brush=None)
                    self.plots[named_value.name].setDownsampling(method='peak')
                    self.data_x[named_value.name] = deque([], 500)
                    self.data_y[named_value.name] = deque([], 500)
                    self.legend.addItem(self.plots[named_value.name], named_value.name)

                self.data_x[named_value.name].append(time.time() - self.time)
                self.data_y[named_value.name].append(named_value.value)
                self.plots[named_value.name].setData(self.data_x[named_value.name], self.data_y[named_value.name])
                self.win.setRange(yRange=[0,100], xRange=[time.time() - self.time - 10, time.time() - self.time], disableAutoRange=True)
            except queue.Empty as empty:
                return