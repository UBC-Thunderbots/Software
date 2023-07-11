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
    def __init__(self, buffer_size=5):
        GLLayer.__init__(self)

        self.primitive_set_buffer = ThreadSafeBuffer(buffer_size, PrimitiveSet)

        self.path_lines = []
        self.desired_position_outlines = []

    def updateGraphics(self):

        if not self.isVisible():
            return (
                [],
                self.clearGraphicsList(self.path_lines)
                + self.clearGraphicsList(self.desired_position_outlines),
            )

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

        added_path_line_graphics, removed_path_line_graphics = self.setupGraphicsList(
            graphics_list=self.path_lines,
            num_graphics=len(paths),
            graphic_init_func=lambda: GLLinePlotItem(),
        )

        added_outline_graphics, removed_outline_graphics = self.setupGraphicsList(
            graphics_list=self.desired_position_outlines,
            num_graphics=len(requested_destinations),
            graphic_init_func=lambda: GLRobotOutline(
                color=Colors.DESIRED_ROBOT_LOCATION_OUTLINE
            ),
        )

        added_graphics = added_path_line_graphics + added_outline_graphics
        removed_graphics = removed_path_line_graphics + removed_outline_graphics

        for index, path in enumerate(paths):
            path_line = self.path_lines[index]
            path_line.setData(
                pos=np.array(
                    [[point.x_meters, point.y_meters, 0] for point in path.points]
                ),
                color=Colors.NAVIGATOR_PATH_COLOR,
            )

        for (dest, final_angle), outline in zip(
            requested_destinations, self.desired_position_outlines
        ):
            outline.setPosition(dest.x_meters, dest.y_meters)
            outline.setOrientation(final_angle.radians)

        return added_graphics, removed_graphics
