from pyqtgraph.opengl import *

import time
import queue
import numpy as np

from proto.world_pb2 import World
from proto.visualization_pb2 import CostVisualization

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.gl.graphics.gl_heatmap import GLHeatmap


class GLCostVisLayer(GLLayer):
    """GLLayer that visualizes pass cost data as a heatmap.
    
    This layer enables us to sample different pass cost functions
    in the field and visualize them.

    WARNING: This layer may slow down AI significantly if the number of points 
    sampled is too high.

    WARNING: The checkbox for generate_sample_passes should only be checked
    when this layer is shown. Otherwise the values will be calculated and
    protobuf data will be sent but not plotted.
    """

    COST_VISUALIZATION_TIMEOUT_S = 0.5

    def __init__(self, name: str, buffer_size: int = 5) -> None:
        """Initialize the GLCostVisLayer

        :param name: The displayed name of the layer
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary
                            
        """
        super().__init__(name)

        self.world_buffer = ThreadSafeBuffer(buffer_size, World)
        self.cost_visualization_buffer = ThreadSafeBuffer(
            buffer_size, CostVisualization
        )
        self.cached_world = World()
        self.cached_cost_vis = CostVisualization()
        self.timeout = (
            time.time() + GLCostVisLayer.COST_VISUALIZATION_TIMEOUT_S
        )

        self.heatmap_graphic = GLHeatmap(parent_item=self)

    def refresh_graphics(self) -> None:
        """Update graphics in this layer"""

        self.cached_world = self.world_buffer.get(block=False)
        field = self.cached_world.field

        try:
            cost_vis = self.cost_visualization_buffer.queue.get_nowait()
        except queue.Empty as empty:
            cost_vis = None

        if not cost_vis:
            cost_vis = self.cached_cost_vis

            # If we haven't received cost visualizations for a bit, hide the heatmap
            if time.time() > self.timeout:
                self.heatmap_graphic.hide()
                return
        else:
            # We received new cost vis data, so lets update our timeout
            self.timeout = (
                time.time() + GLCostVisLayer.COST_VISUALIZATION_TIMEOUT_S
            )
            self.cached_cost_vis = cost_vis

        # Cost vis data is in column-major order; reshape into 2D matrix
        data = np.array(cost_vis.cost).reshape(cost_vis.num_rows, cost_vis.num_cols, order="F")

        self.heatmap_graphic.set_dimensions(field.field_x_length, field.field_y_length)
        self.heatmap_graphic.setData(data)
        self.heatmap_graphic.show()
