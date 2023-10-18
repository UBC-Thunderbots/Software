from pyqtgraph.opengl import *

import time
import queue

import numpy as np

from proto.visualization_pb2 import PassVisualization

from software.thunderscope.constants import Colors, LINE_WIDTH
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.gl.graphics.gl_polygon import GLPolygon

from software.thunderscope.gl.helpers.observable_list import ObservableList


class GLPassingLayer(GLLayer):
    """GLLayer that visualizes passes"""

    # The pass generator is created and destroyed as we move in and out of offensive plays.
    # If we no longer receive new passes, we need to remove the old one.
    PASS_VISUALIZATION_TIMEOUT_S = 0.5

    # The number of passes to show in the visualization
    NUM_PASSES_TO_SHOW = 1

    def __init__(self, name: str, buffer_size: int = 5) -> None:
        """Initialize the GLPassingLayer

        :param name: The displayed name of the layer
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary
                            
        """
        super().__init__(name)

        self.pass_visualization_buffer = ThreadSafeBuffer(
            buffer_size, PassVisualization
        )
        self.cached_pass_vis = PassVisualization()
        self.timeout = time.time() + GLPassingLayer.PASS_VISUALIZATION_TIMEOUT_S

        self.pass_graphics = ObservableList(self._graphics_changed)

    def refresh_graphics(self) -> None:
        """Update graphics in this layer"""

        try:
            pass_vis = self.pass_visualization_buffer.queue.get_nowait()
        except queue.Empty as empty:
            pass_vis = None

        if not pass_vis:
            pass_vis = self.cached_pass_vis

            # If we haven't received pass visualizations for a bit, skip updating graphics
            if time.time() > self.timeout:
                return
        else:
            # We received new pass data, so lets update our timeout
            self.timeout = time.time() + GLPassingLayer.PASS_VISUALIZATION_TIMEOUT_S
            self.cached_pass_vis = pass_vis

        sorted_pass_with_rating = sorted(
            pass_vis.best_passes, key=lambda x: x.rating, reverse=True
        )
        passes_to_show = sorted_pass_with_rating[0 : GLPassingLayer.NUM_PASSES_TO_SHOW]

        # Ensure we have the same number of graphics as protos
        self.pass_graphics.resize(
            len(passes_to_show),
            lambda: GLPolygon(
                outline_color=Colors.PASS_VISUALIZATION_COLOR, line_width=LINE_WIDTH
            ),
        )

        for pass_graphic, pass_with_rating in zip(self.pass_graphics, passes_to_show,):
            pass_graphic.set_points(
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
            )
