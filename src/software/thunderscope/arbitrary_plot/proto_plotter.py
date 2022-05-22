import time

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl

from proto.visualization_pb2 import NamedValue
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt import QtGui, QtCore

from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

DEQUE_SIZE = 1000
MIN_Y_RANGE = 0
MAX_Y_RANGE = 100
TIME_WINDOW_TO_DISPLAY_S = 20


class GLViewWidget2DPlot(gl.GLViewWidget):

    """Limit the mouse controls and fix the camera on a 3D
    view to make it 2D"""

    def __init__(self, width, min_y, max_y, grid_resolution):
        """Initialize the GLViewWidget2DPlot

        :param width: The width of the plot
        :param min_y: Minimum y value to display
        :param max_y: Maximum y value to display
        :param grid_resolution: Resolution of the grid

        """
        gl.GLViewWidget.__init__(self)

        self.setMinimumSize(width, 100)
        self.setCameraPosition(distance=2000, elevation=90, azimuth=0)

        self.grid = gl.GLGridItem()
        self.grid.setSize(10, 10)
        self.grid.scale(width, max_y - min_y, 1)
        self.grid.setSpacing(x=grid_resolution, y=grid_resolution)
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
        self, width, min_y, max_y, grid_resolution, configuration, buffer_size=1000
    ):
        """Initializes NamedValuePlotter.

        :param width: The width of the plot
        :param min_y: Minimum y value to display
        :param max_y: Maximum y value to display
        :param grid_resolution: Resolution of the grid
        :param configuration: A dictionary of protobuf types to data extractor
        :param buffer_size: The size of the buffer to use for plotting.

        """
        QWidget.__init__(self)

        self.traces = {}
        self.data = {}

        self.layout = QHBoxLayout()
        self.plot = GLViewWidget2DPlot(width, min_y, max_y, grid_resolution)

        self.buffers = {
            key: ThreadSafeBuffer(buffer_size, key) for key in configuration.keys()
        }

        self.select_plots = QListWidget()
        self.select_plots.setMaximumWidth(self.select_plots.sizeHint().width())

        self.layout.addWidget(self.select_plots)
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

        self.configuration = configuration

    def refresh(self):
        """Refreshes NamedValuePlotter and updates data in the respective
        plots.

        """
        for proto_class, buffer in self.buffers.items():
            for _ in range(buffer.queue.qsize()):

                data = self.configuration[proto_class](buffer.get(block=False))

                for name, value in data.items():
                    if name not in self.traces:

                        item = QListWidgetItem(name)
                        item.setFlags(
                            item.flags() | QtCore.Qt.ItemFlag.ItemIsUserCheckable
                        )
                        item.setCheckState(QtCore.Qt.CheckState.Checked)

                        self.select_plots.addItem(item)
                        self.select_plots.setMaximumWidth(
                            self.select_plots.sizeHint().width()
                        )

                        self.data[name] = np.zeros(DEQUE_SIZE)

                        self.color += 1
                        self.traces[name] = gl.GLLinePlotItem(
                            color=pg.glColor(self.color), width=1, antialias=True,
                        )
                        self.plot.addItem(self.traces[name])

                    # Add incoming data to existing deques of data
                    self.last_incoming_value[name] = -value

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
