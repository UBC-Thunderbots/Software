import random
import time
from collections import deque

import pyqtgraph as pg
import numpy as np
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

    def __init__(self, configuration, buffer_size=50):
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

        # 1) Simplest approach -- update data in the array such that plot appears to scroll
        #    In these examples, the array size is fixed.
        self.data1 = np.random.normal(size=1000)
        self.data2 = np.random.normal(size=1000)
        self.data3 = np.random.normal(size=1000)
        self.data4 = np.random.normal(size=1000)
        self.data5 = np.random.normal(size=1000)
        self.data6 = np.random.normal(size=1000)
        self.data7 = np.random.normal(size=1000)
        self.curve1 = self.win.plot(self.data1, 
                                    pen=QtGui.QColor(
                                        random.randint(100, 255),
                                        random.randint(100, 255),
                                        random.randint(100, 255),
                                    )
                )
        self.curve2 = self.win.plot(self.data2,
                                    pen=QtGui.QColor(
                                        random.randint(100, 255),
                                        random.randint(100, 255),
                                        random.randint(100, 255),
                                    )

                )
        self.curve3 = self.win.plot(self.data3,

                                    pen=QtGui.QColor(
                                        random.randint(100, 255),
                                        random.randint(100, 255),
                                        random.randint(100, 255),
                                    ))
        self.curve4 = self.win.plot(self.data4, 
                                    pen=QtGui.QColor(
                                        random.randint(100, 255),
                                        random.randint(100, 255),
                                        random.randint(100, 255),
                                    ))
        self.curve5 = self.win.plot(self.data5, 
                                    pen=QtGui.QColor(
                                        random.randint(100, 255),
                                        random.randint(100, 255),
                                        random.randint(100, 255),
                                    ))
        self.curve6 = self.win.plot(self.data6, 
                                    pen=QtGui.QColor(
                                        random.randint(100, 255),
                                        random.randint(100, 255),
                                        random.randint(100, 255),
                                    ))
        self.curve7 = self.win.plot(self.data7,
                                    pen=QtGui.QColor(
                                        random.randint(100, 255),
                                        random.randint(100, 255),
                                        random.randint(100, 255),
                                    ))

    def update1(self):
        self.data1[:-1] = self.data1[1:]  # shift self.data in the array one sample left
        self.data2[:-1] = self.data2[1:]  # shift self.data in the array one sample left
        self.data3[:-1] = self.data3[1:]  # shift self.data in the array one sample left
        self.data4[:-1] = self.data4[1:]  # shift self.selfself..data in the array one sample left
        self.data5[:-1] = self.data5[1:]  # shift data in the array one sample left
        self.data6[:-1] = self.data6[1:]  # shift data in the array one sample left
        self.data7[:-1] = self.data7[1:]  # shift data in the array one sample left
                                # (see also: np.roll)
        self.data1[-1] = np.random.normal()
        self.data2[-1] = np.random.normal()
        self.data3[-1] = np.random.normal()
        self.data4[-1] = np.random.normal()
        self.data5[-1] = np.random.normal()
        self.data6[-1] = np.random.normal()
        self.data7[-1] = np.random.normal()

        start_time = time.time()
        self.curve1.setData(self.data1)
        self.curve2.setData(self.data2)
        self.curve3.setData(self.data3)
        self.curve4.setData(self.data4)
        self.curve5.setData(self.data5)
        self.curve6.setData(self.data6)
        self.curve7.setData(self.data7)
        end_time = time.time()
        print(end_time - start_time)

    def refresh(self):
        """Refreshes ProtoPlotter and updates data in the respective
        plots.

        """
        self.update1()
