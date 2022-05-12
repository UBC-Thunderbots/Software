import random
import time
import numpy as np

import pyqtgraph as pg
import pyqtgraph.opengl as gl

from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt import QtGui, QtCore

from proto.visualization_pb2 import NamedValue

from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

DEQUE_SIZE = 1000
MIN_Y_RANGE = 0
MAX_Y_RANGE = 100
TIME_WINDOW_TO_DISPLAY_S = 20


class NamedValuePlotter(QWidget):

    """ Plot named values in real time with a scrolling plot """

    def __init__(self, buffer_size=1000):
        """Initializes NamedValuePlotter.

        :param buffer_size: The size of the buffer to use for plotting.

        """
        QWidget.__init__(self)

        self.layout = QHBoxLayout()

        self.plots = {}
        self.data_x = {}
        self.data_y = {}
        self.time = time.time()
        self.named_value_buffer = ThreadSafeBuffer(buffer_size, NamedValue)

        self.traces = {}
        self.win = gl.GLViewWidget()
        self.win.setMinimumSize(100, 100)
        self.win.setCameraPosition(distance=100, elevation=90, azimuth=0)

        # Do NOT allow orbit
        self.win.orbit = lambda x, y: False

        self.g = gl.GLGridItem()
        self.g.setSize(x=20, y=20, z=10)
        self.g.scale(50, 50, 10)
        self.win.addItem(self.g)

        self.layout.addWidget(self.win)
        self.setLayout(self.layout)

        self.last_update_time = time.time()
        self.last_incoming_value = {}

    def refresh(self):
        """Refreshes NamedValuePlotter and updates data in the respective
        plots.

        """
        for _ in range(self.named_value_buffer.queue.qsize()):

            named_value = self.named_value_buffer.get(block=False)

            if named_value.name not in self.traces:
                self.last_incoming_value[named_value.name] = named_value.value
                self.data_y[named_value.name] = np.zeros(DEQUE_SIZE)

                self.traces[named_value.name] = gl.GLLinePlotItem(
                    color=pg.glColor(
                        random.randint(0, 255),
                        random.randint(0, 255),
                        random.randint(0, 255),
                    ),
                    width=1,
                    antialias=True,
                )
                self.win.addItem(self.traces[named_value.name])

            # Add incoming data to existing deques of data
            self.last_incoming_value[named_value.name] = named_value.value

        if self.last_update_time + 0.01 > time.time():
            return

        self.last_update_time = time.time()

        for name, trace in self.traces.items():

            self.data_y[name][0:-1] = self.data_y[name][1:]
            self.data_y[name][-1] = self.last_incoming_value[name]

            trace.setData(
                pos=np.vstack(
                    [
                        self.data_y[name],
                        np.array(range(int(-DEQUE_SIZE / 2), int(DEQUE_SIZE / 2))),
                        np.zeros(len(self.data_y[name])),
                    ]
                ).transpose()
            )
