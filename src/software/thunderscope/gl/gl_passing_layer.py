import time
import queue

from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.opengl import *

import numpy as np

from proto.import_all_protos import *
from proto.tbots_software_msgs_pb2 import PrimitiveSet

import software.thunderscope.constants as constants
from software.py_constants import *
from software.thunderscope.constants import Colors
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.gl.gl_layer import GLLayer

class GLPassingLayer(GLLayer):

    # The pass generator is created and destroyed as we move in and out of offensive plays.
    # If we no longer receive new passes, we need to remove the old one.
    PASS_VISUALIZATION_TIMEOUT_S = 0.5

    # The number of passes to show in the visualization
    NUM_PASSES_TO_SHOW = 1

    def __init__(self, buffer_size=5):
        GLLayer.__init__(self)

        self.pass_visualization_buffer = ThreadSafeBuffer(
            buffer_size, PassVisualization
        )
        self.cached_pass_vis = PassVisualization()
        self.timeout = time.time() + GLPassingLayer.PASS_VISUALIZATION_TIMEOUT_S

        self.pass_lines = []

    def updateGraphics(self):

        added_graphics = []
        removed_graphics = []

        if not self.isVisible():
            for index in range(len(self.pass_lines)):
                removed_graphics.append(self.pass_lines.pop())
            return added_graphics, removed_graphics

        try:
            pass_vis = self.pass_visualization_buffer.queue.get_nowait()
        except queue.Empty as empty:
            pass_vis = None

        if not pass_vis:
            pass_vis = self.cached_pass_vis

            # If we haven't received pass visualizations for a bit, clear the layer's graphics
            if time.time() > self.timeout:
                for index in range(len(self.pass_lines)):
                    removed_graphics.append(self.pass_lines.pop())
                return added_graphics, removed_graphics
        else:
            # We received new pass data, so lets update our timeout
            self.timeout = time.time() + GLPassingLayer.PASS_VISUALIZATION_TIMEOUT_S
            self.cached_pass_vis = pass_vis

        sorted_pass_with_rating = sorted(
            pass_vis.best_passes, key=lambda x: x.rating, reverse=True
        )
        passes_to_show = sorted_pass_with_rating[0:GLPassingLayer.NUM_PASSES_TO_SHOW]
        
        # Add or remove GLLinePlotItems from self.pass_lines so that we
        # have the same number of GLLinePlotItems as passes
        if len(self.pass_lines) < len(passes_to_show):
            num_lines_to_add = len(passes_to_show) - len(self.pass_lines)

            for i in range(num_lines_to_add):
                gl_line_plot_item = GLLinePlotItem()
                self.pass_lines.append(gl_line_plot_item)
                added_graphics.append(gl_line_plot_item)

        elif len(self.pass_lines) > len(passes_to_show):
            num_lines_to_remove = len(self.pass_lines) - len(passes_to_show)

            for i in range(num_lines_to_remove):
                removed_graphics.append(self.pass_lines.pop())

        for index, pass_with_rating in enumerate(passes_to_show):
            pass_line = self.pass_lines[index]
            pass_line.setData(
                pos=np.array([
                    [
                        pass_with_rating.pass_.passer_point.x_meters,
                        pass_with_rating.pass_.passer_point.y_meters,
                    ],
                    [
                        pass_with_rating.pass_.receiver_point.x_meters,
                        pass_with_rating.pass_.receiver_point.y_meters
                    ]
                ]),
                color=Colors.PASS_VISUALIZATION_COLOR
            )

        return added_graphics, removed_graphics

        