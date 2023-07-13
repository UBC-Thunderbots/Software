from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.opengl import *

import numpy as np

from proto.tbots_software_msgs_pb2 import PrimitiveSet

import software.thunderscope.constants as constants
from software.py_constants import *
from software.thunderscope.constants import Colors
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.gl.gl_layer import GLLayer
from software.thunderscope.gl.graphics.gl_robot_outline import GLRobotOutline


class GLPathLayer(GLLayer):
    """GLLayer that visualizes paths from the navigator"""

    def __init__(self, buffer_size=5):
        """Initialize the GLPathLayer

        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary
                            
        """
        GLLayer.__init__(self)

        self.primitive_set_buffer = ThreadSafeBuffer(buffer_size, PrimitiveSet)

        self.graphics_list.registerGraphicsGroup("paths", GLLinePlotItem)
        self.graphics_list.registerGraphicsGroup(
            "destinations",
            lambda: GLRobotOutline(color=Colors.DESIRED_ROBOT_LOCATION_OUTLINE),
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
            self.graphics_list.getGraphics("paths", len(paths)), paths
        ):
            path_graphic.setData(
                pos=np.array(
                    [[point.x_meters, point.y_meters, 0] for point in path.points]
                ),
                color=Colors.NAVIGATOR_PATH_COLOR,
            )

        for dest_graphic, (dest, final_angle) in zip(
            self.graphics_list.getGraphics("destinations", len(requested_destinations)),
            requested_destinations,
        ):
            dest_graphic.setPosition(dest.x_meters, dest.y_meters)
            dest_graphic.setOrientation(final_angle.radians)

        return self.graphics_list.getChanges()
