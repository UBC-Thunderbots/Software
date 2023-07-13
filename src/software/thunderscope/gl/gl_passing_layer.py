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
    """GLLayer that visualizes passes"""

    # The pass generator is created and destroyed as we move in and out of offensive plays.
    # If we no longer receive new passes, we need to remove the old one.
    PASS_VISUALIZATION_TIMEOUT_S = 0.5

    # The number of passes to show in the visualization
    NUM_PASSES_TO_SHOW = 1

    def __init__(self, buffer_size=5):
        """Initialize the GLPassingLayer

        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary
                            
        """
        GLLayer.__init__(self)

        self.pass_visualization_buffer = ThreadSafeBuffer(
            buffer_size, PassVisualization
        )
        self.cached_pass_vis = PassVisualization()
        self.timeout = time.time() + GLPassingLayer.PASS_VISUALIZATION_TIMEOUT_S

        self.graphics_list.registerGraphicsGroup(
            "passes", lambda: GLLinePlotItem(color=Colors.PASS_VISUALIZATION_COLOR)
        )

    def updateGraphics(self):
        """Update the GLGraphicsItems in this layer

        :returns: tuple (added_graphics, removed_graphics)
            - added_graphics - List of the added GLGraphicsItems
            - removed_graphics - List of the removed GLGraphicsItems
        
        """
        # Clear all graphics in this layer if not visible
        if not self.isVisible():
            return self.graphics_list.getChanges()

        try:
            pass_vis = self.pass_visualization_buffer.queue.get_nowait()
        except queue.Empty as empty:
            pass_vis = None

        if not pass_vis:
            pass_vis = self.cached_pass_vis

            # If we haven't received pass visualizations for a bit, clear the layer's graphics
            if time.time() > self.timeout:
                return self.graphics_list.getChanges()
        else:
            # We received new pass data, so lets update our timeout
            self.timeout = time.time() + GLPassingLayer.PASS_VISUALIZATION_TIMEOUT_S
            self.cached_pass_vis = pass_vis

        sorted_pass_with_rating = sorted(
            pass_vis.best_passes, key=lambda x: x.rating, reverse=True
        )
        passes_to_show = sorted_pass_with_rating[0 : GLPassingLayer.NUM_PASSES_TO_SHOW]

        for pass_graphic, pass_with_rating in zip(
            self.graphics_list.getGraphics("passes", len(passes_to_show)),
            passes_to_show,
        ):
            pass_graphic.setData(
                pos=np.array(
                    [
                        [
                            pass_with_rating.pass_.passer_point.x_meters,
                            pass_with_rating.pass_.passer_point.y_meters,
                        ],
                        [
                            pass_with_rating.pass_.receiver_point.x_meters,
                            pass_with_rating.pass_.receiver_point.y_meters,
                        ],
                    ]
                ),
            )

        return self.graphics_list.getChanges()
