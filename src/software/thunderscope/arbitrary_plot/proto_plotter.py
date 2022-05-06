import random
import time
from collections import deque

from time import perf_counter
import numpy as np
import pyqtgraph as pg
from proto.visualization_pb2 import NamedValue
from pyqtgraph.Qt import QtGui

from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

DEQUE_SIZE = 1000
MIN_Y_RANGE = 0
MAX_Y_RANGE = 100
TIME_WINDOW_TO_DISPLAY_S = 20


class ProtoPlotter(object):

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

    def __init__(self, configuration, buffer_size=500):
        """Initializes ProtoPlotter.

        :param configuration: A dictionary that maps protobuf types to
                              data extractor functions. See class docstring.
        :param buffer_size: The size of the buffer to use for plotting.

        """
        self.time = time.time()
        self.win = pg.plot()
        self.plots = {}
        self.data_x = {}
        self.data_y = {}
        self.legend = pg.LegendItem((80, 60), offset=(70, 20))
        self.legend.setParentItem(self.win.graphicsItem())

        self.configuration = configuration

        self.buffers = {
            key: ThreadSafeBuffer(buffer_size, key) for key in configuration.keys()
        }

        rollingAverageSize = 1000
        self.elapsed = deque(maxlen=rollingAverageSize)
        self.fpsLastUpdate = perf_counter()

    def refresh(self):
        """Refreshes ProtoPlotter and updates data in the respective
        plots.

        """
        t_start = perf_counter()

        # Dump the entire buffer into a deque. This operation is fast because
        # its just consuming data from the buffer and appending it to a deque.
        for proto_class, buffer in self.buffers.items():
            for _ in range(buffer.queue.qsize()):

                data = self.configuration[proto_class](buffer.get(block=False))

                for name, value in data.items():
                    if name not in self.plots:

                        self.plots[name] = self.win.plot(
                            pen=QtGui.QColor(
                                random.randint(100, 255),
                                random.randint(100, 255),
                                random.randint(100, 255),
                            ),
                            name=name,
                            disableAutoRange=True,
                            brush=None,
                            skipFiniteCheck=True,
                        )

                        self.plots[name].setDownsampling(method="subsample")
                        self.data_x[name] = np.zeros(DEQUE_SIZE)
                        self.data_y[name] = np.zeros(DEQUE_SIZE)
                        self.legend.addItem(self.plots[name], name)

                    self.data_x[name][0:-1] = self.data_x[name][1:]
                    self.data_y[name][0:-1] = self.data_y[name][1:]

                    self.data_x[name][-1] = time.time() - self.time
                    self.data_y[name][-1] = value

        # Update the data
        for name, plot in self.plots.items():
            plot.setData(self.data_x[name], self.data_y[name])

        self.win.setRange(
            xRange=[
                time.time() - self.time - TIME_WINDOW_TO_DISPLAY_S,
                time.time() - self.time,
            ],
        )

        t_end = perf_counter()

        # update fps at most once every 0.2 secs
        if t_end - self.fpsLastUpdate > 0.2:
            self.fpsLastUpdate = t_end
            average = np.mean(self.elapsed)
            fps = 1 / average
            self.win.setTitle('%0.2f fps - %0.1f ms avg' % (fps, average * 1_000))

        self.elapsed.append(t_end - t_start)
