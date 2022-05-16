import time
import numpy as np

import pyqtgraph as pg
import pyqtgraph.opengl as gl

from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt import QtGui, QtCore

from proto.visualization_pb2 import NamedValue

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

DEQUE_SIZE = 1000


class GLViewWidget2DPlot(gl.GLViewWidget):

    """Limit the mouse controls and fix the camera on a 3D
    view to make it 2D"""

    def __init__(self):
        gl.GLViewWidget.__init__(self)

        self.setMinimumSize(200, 100)
        self.setCameraPosition(distance=2000, elevation=90, azimuth=0)

        self.grid = gl.GLGridItem()
        self.grid.setSize(x=20, y=40, z=10)
        self.grid.scale(100, 100, 10)
        self.addItem(self.grid)

    def mouseMoveEvent(self, event):
        """Overridden

        We want to disable rotation and only pan the screen

        :param event: The event to handle

        """
        diff = event.position() - self.mousePos
        self.mousePos = event.position()

        if event.buttons() == QtCore.Qt.MouseButton.LeftButton:
            self.pan(diff.x(), diff.y(), 0, relative="view")


class NamedValuePlotter(QWidget):

    """ Plot named values in real time with a scrolling plot """

    def __init__(self, buffer_size=1000):
        """Initializes NamedValuePlotter.

        :param buffer_size: The size of the buffer to use for plotting.

        """
        QWidget.__init__(self)

        self.layout = QHBoxLayout()
        self.data = {}
        self.named_value_buffer = ThreadSafeBuffer(buffer_size, NamedValue)

        self.traces = {}
        self.plot = GLViewWidget2DPlot()

        self.layout.addWidget(self.plot)
        self.setLayout(self.layout)

        self.last_update_time = time.time()
        self.last_incoming_value = {}
        self.color = 0

        self.txtitem2 = gl.GLTextItem()
        self.txtitem2.setData(
            pos=(1.0, -1.0, 2.0), color=(127, 255, 127, 255), text="text2"
        )
        self.plot.addItem(self.txtitem2)

    def refresh(self):
        """Refreshes NamedValuePlotter and updates data in the respective
        plots.

        """
        for _ in range(self.named_value_buffer.queue.qsize()):

            named_value = self.named_value_buffer.get(block=False)

            if named_value.name not in self.traces:

                self.last_incoming_value[named_value.name] = named_value.value
                self.data[named_value.name] = np.zeros(DEQUE_SIZE)

                self.color += 1
                self.traces[named_value.name] = gl.GLLinePlotItem(
                    color=pg.glColor(self.color), width=1, antialias=True,
                )
                self.plot.addItem(self.traces[named_value.name])

            # Add incoming data to existing deques of data
            self.last_incoming_value[named_value.name] = -named_value.value

        if self.last_update_time + 0.02 > time.time():
            return

        self.last_update_time = time.time()

        for name, trace in self.traces.items():

            self.data[name][:-1] = self.data[name][1:]
            self.data[name][-1] = self.last_incoming_value[name]

            trace.setData(
                pos=np.vstack(
                    [
                        self.data[name],
                        np.array(range(int(-DEQUE_SIZE / 2), int(DEQUE_SIZE / 2))),
                        np.zeros(len(self.data[name])),
                    ]
                ).transpose()
            )
