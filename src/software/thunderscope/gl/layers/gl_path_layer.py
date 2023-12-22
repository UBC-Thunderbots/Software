from pyqtgraph.opengl import *

import math

from proto.tbots_software_msgs_pb2 import PrimitiveSet
from proto.visualization_pb2 import PathVisualization

from software.thunderscope.constants import Colors, DepthValues
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.gl.graphics.gl_robot_outline import GLRobotOutline
from software.thunderscope.gl.graphics.gl_polygon import GLPolygon

from software.thunderscope.gl.helpers.observable_list import ObservableList


class GLPathLayer(GLLayer):
    """GLLayer that visualizes paths from the navigator"""

    def __init__(self, name: str, buffer_size: int = 5) -> None:
        """Initialize the GLPathLayer

        :param name: The displayed name of the layer
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary
                            
        """
        super().__init__(name)
        self.setDepthValue(DepthValues.BACKGROUND_DEPTH)

        self.primitive_set_buffer = ThreadSafeBuffer(buffer_size, PrimitiveSet)
        self.path_visualization_buffer = ThreadSafeBuffer(buffer_size, PathVisualization)

        self.destination_graphics = ObservableList(self._graphics_changed)
        self.path_graphics = ObservableList(self._graphics_changed)

    def refresh_graphics(self) -> None:
        """Update graphics in this layer"""

        path_list = self.path_visualization_buffer.get(block=False).paths
        primitive_set = self.primitive_set_buffer.get(
            block=False
        ).robot_primitives.values()

        paths = [
            path for path in path_list
        ]
        requested_destinations = [
            (
                primitive.move.xy_traj_params.destination,
                primitive.move.w_traj_params.final_angle,
            )
            for primitive in primitive_set
            if primitive.HasField("move")
        ]

        # Ensure we have the same number of graphics as protos
        self.path_graphics.resize(
            len(paths), lambda: GLPolygon(outline_color=Colors.NAVIGATOR_PATH_COLOR),
        )
        self.destination_graphics.resize(
            len(requested_destinations),
            lambda: GLRobotOutline(outline_color=Colors.DESIRED_ROBOT_LOCATION_OUTLINE),
        )

        # Visualize path and the desired destination
        for path_graphic, path in zip(self.path_graphics, paths):
            path_graphic.set_points(
                [[point.x_meters, point.y_meters] for point in path.points]
            )

        for dest_graphic, (dest, final_angle) in zip(
            self.destination_graphics, requested_destinations
        ):
            dest_graphic.set_position(dest.x_meters, dest.y_meters)
            dest_graphic.set_orientation(math.degrees(final_angle.radians))
