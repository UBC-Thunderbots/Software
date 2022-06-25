import time
import random

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl

from proto.visualization_pb2 import NamedValue
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt import QtGui, QtCore

from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class GLViewWidget2DPlot(gl.GLViewWidget):

    """Limit the mouse controls and fix the camera on a 3D
    view to make it 2D"""

    def __init__(self, grid_cell_size, grid_width, grid_height):
        """Initialize the GLViewWidget2DPlot

        :param grid_cell_size: The size of each grid cell
        :param grid_width: The number of grid cells in the x direction
        :param grid_height: The number of grid cells in the y direction

        """
        gl.GLViewWidget.__init__(self)

        # The GL View Widget is a 3D widget, but we use a plane to display a 2D
        # plot. To see the entire plot, we need to set the distance high enough
        # that the entire grid is in view.
        self.setCameraPosition(
            distance=grid_cell_size * grid_width, elevation=90, azimuth=0,
        )
        self.grid = gl.GLGridItem()
        self.grid.setSize(grid_height, grid_width)
        self.grid.scale(grid_cell_size, grid_cell_size, 1)
        self.addItem(self.grid)

    def mouseMoveEvent(self, event):
        """Overridden to do nothing. We don't want to move the camera
        because we can't 

        TODO (#2634) Implement dynamic grid scaling and axis adjustments
        and allow panning when the plots been zoomed without "falling off"
        the grid.

        """

    def wheelEvent(self, event):
        """Overridden to do nothing

        TODO (#2634) Implement dynamic grid scaling and axis adjustments

        """


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

        self.traces = {}
        self.data = {}

        self.width = 100
        self.main_layout = QHBoxLayout()
        self.plot_selection_and_settings_layout = QVBoxLayout()

        self.plot = GLViewWidget2DPlot(
            grid_cell_size=self.width, grid_width=10, grid_height=10
        )

        self.buffers = {
            key: ThreadSafeBuffer(buffer_size, key) for key in configuration.keys()
        }

        self.last_update_time = time.time()
        self.last_incoming_value = {}

        self.configuration = configuration
        self.deque_size = int(window_secs * plot_rate_hz)
        self.update_interval = 1.0 / plot_rate_hz

        self.graph_settings_layout = QVBoxLayout()

        # Select Min Y range
        self.min_y_spinbox = QSpinBox()
        self.min_y_spinbox.setRange(-1000, 1000)
        self.min_y_spinbox.setSingleStep(10)
        self.min_y_spinbox.setSuffix(" y-min")

        # Select Max Y range
        self.max_y_spinbox = QSpinBox()
        self.max_y_spinbox.setRange(-1000, 1000)
        self.max_y_spinbox.setSingleStep(10)
        self.max_y_spinbox.setSuffix(" y-max")

        # Select X axis range in seconds
        self.window_spinbox = QSpinBox()
        self.window_spinbox.setRange(5, 60)
        self.window_spinbox.setSingleStep(1)
        self.window_spinbox.setSuffix(" sec")

        # Settings for graph
        self.graph_settings_layout.addWidget(self.min_y_spinbox)
        self.graph_settings_layout.addWidget(self.max_y_spinbox)
        self.graph_settings_layout.addWidget(self.window_spinbox)

        # Plot Selector Widget (Legend)
        self.select_plots = QVBoxLayout()

        # Setup Layout
        self.plot_selection_and_settings_layout.addLayout(self.select_plots)
        self.plot_selection_and_settings_layout.addLayout(self.graph_settings_layout)
        self.main_layout.addLayout(self.plot_selection_and_settings_layout, stretch=1)
        self.main_layout.addWidget(self.plot, stretch=10)
        self.setLayout(self.main_layout)

    def refresh(self):
        """Refreshes NamedValuePlotter and updates data in the respective
        plots.

        """
        for proto_class, buffer in self.buffers.items():
            for _ in range(buffer.queue.qsize()):

                data = self.configuration[proto_class](buffer.get(block=False))

                for name, value in data.items():
                    if name not in self.traces:

                        # Pick a random color
                        r = random.randint(100, 255)
                        g = random.randint(100, 255)
                        b = random.randint(100, 255)

                        item = QCheckBox(name)
                        item.setCheckState(QtCore.Qt.CheckState.Checked)
                        item.setStyleSheet(f"QCheckBox {{ color: rgb({r}, {g}, {b}) }}")
                        self.select_plots.addWidget(item)

                        self.data[name] = np.zeros(self.deque_size)

                        self.traces[name] = gl.GLLinePlotItem(
                            color=pg.glColor(r, g, b), width=1, antialias=True,
                        )
                        self.plot.addItem(self.traces[name])

                    # Add incoming data to existing deques of data
                    self.last_incoming_value[name] = -value

        if self.last_update_time + self.update_interval > time.time():
            return

        self.last_update_time = time.time()

        for name, trace in self.traces.items():

            self.data[name][:-1] = self.data[name][1:]
            self.data[name][-1] = self.last_incoming_value[name]

            trace.setData(
                pos=np.vstack(
                    [
                        self.data[name],
                        np.array(
                            np.linspace(
                                -self.width * 5, self.width * 5, self.deque_size
                            )
                        ),
                        np.zeros(len(self.data[name])),
                    ]
                ).transpose()
            )
