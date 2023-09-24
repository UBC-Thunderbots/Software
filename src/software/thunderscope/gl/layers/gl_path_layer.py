from pyqtgraph.opengl import *

import math
import numpy as np

from proto.tbots_software_msgs_pb2 import PrimitiveSet

from software.thunderscope.constants import Colors
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.gl.graphics.gl_robot_outline import GLRobotOutline


class GLPathLayer(GLLayer):
    """GLLayer that visualizes paths from the navigator"""

    def __init__(self, name: str, buffer_size: int = 5):
        """Initialize the GLPathLayer

        :param name: The displayed name of the layer
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary
                            
        """
        GLLayer.__init__(self, name)

        self.primitive_set_buffer = ThreadSafeBuffer(buffer_size, PrimitiveSet)

        self.graphics_list.register_graphics_group("paths", GLLinePlotItem)
        self.graphics_list.register_graphics_group(
            "destinations",
            lambda: GLRobotOutline(color=Colors.DESIRED_ROBOT_LOCATION_OUTLINE),
        )

    def _update_graphics(self):
        """Fetch and update graphics for the layer"""

        primitive_set = self.primitive_set_buffer.get(
            block=False
        ).robot_primitives.values()

        paths = [
            primitive.move.motion_control.path
            for primitive in primitive_set
            if primitive.HasField("move")
        ]

        requested_destinations = [
            (
                primitive.move.motion_control.requested_destination,
                primitive.move.final_angle,
            )
            for primitive in primitive_set
            if primitive.HasField("move")
        ]

        for path_graphic, path in zip(
            self.graphics_list.get_graphics("paths", len(paths)), paths
        ):
            path_graphic.setData(
                pos=np.array(
                    [[point.x_meters, point.y_meters, 0] for point in path.points]
                ),
                color=Colors.NAVIGATOR_PATH_COLOR,
            )

        for dest_graphic, (dest, final_angle) in zip(
            self.graphics_list.get_graphics(
                "destinations", len(requested_destinations)
            ),
            requested_destinations,
        ):
            dest_graphic.set_position(dest.x_meters, dest.y_meters)
            dest_graphic.set_orientation(math.degrees(final_angle.radians))
