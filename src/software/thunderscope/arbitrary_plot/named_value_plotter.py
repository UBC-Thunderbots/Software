import random
import time
import numpy as np
from collections import deque

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

        self.phase = 0
        self.lines = 50
        self.points = 1000
        self.y = np.linspace(-10, 10, self.lines)
        self.x = np.linspace(-10, 10, self.points)

        self.traces = dict()
        self.win = gl.GLViewWidget()
        self.win.setMinimumSize(640, 480)
        self.win.setCameraPosition(distance=100, elevation=90, azimuth=0)

        self.w_gl_axis = gl.GLAxisItem(
            size=None, antialias=True, glOptions='translucent')
        self.win.addItem(self.w_gl_axis)

        self.g = gl.GLGridItem()
        self.g.scale(10, 10, 1)
        self.win.addItem(self.g)

        self.layout.addWidget(self.win)
        self.setLayout(self.layout)

    def refresh(self):
        """Refreshes NamedValuePlotter and updates data in the respective
        plots.

        """
        for _ in range(self.named_value_buffer.queue.qsize()):

            named_value = self.named_value_buffer.get(block=False)

            if named_value.name not in self.traces:
                self.data_x[named_value.name] = np.zeros(DEQUE_SIZE)
                self.data_y[named_value.name] = np.zeros(DEQUE_SIZE)

                self.traces[named_value.name] = gl.GLLinePlotItem(
                    pos=np.vstack(
                        [
                            self.data_y[named_value.name],
                            self.data_x[named_value.name],
                            np.zeros(
                                len(self.data_y[named_value.name])
                            )
                        ]
                    ).transpose(),
                    color=pg.glColor((random.randint(0,10), self.lines * 1.3)),
                    width=1,
                    antialias=True
                )
                self.win.addItem(self.traces[named_value.name])


            # Add incoming data to existing deques of data
            self.data_x[named_value.name][0:-1] = self.data_x[named_value.name][1:]
            self.data_y[named_value.name][0:-1] = self.data_y[named_value.name][1:]

            self.data_x[named_value.name][-1] = (time.time() - self.time) * 10
            self.data_y[named_value.name][-1] = named_value.value

        for name, trace in self.traces.items():
            trace.setData(
                    pos=np.vstack(
                        [
                            self.data_y[name],
                            self.data_x[name],
                            np.zeros(
                                len(self.data_y[name])
                            )
                        ]
                    ).transpose())
