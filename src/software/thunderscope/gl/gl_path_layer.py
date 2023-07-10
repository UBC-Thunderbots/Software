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

        added_graphics = []
        removed_graphics = []

        if not self.isVisible():

            for index in range(len(self.desired_position_outlines)):
                removed_graphics.append(self.desired_position_outlines.pop())

            for index in range(len(self.desired_position_outlines)):
                removed_graphics.append(self.desired_position_outlines.pop())

            return added_graphics, removed_graphics

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

        # Add or remove GLLinePlotItems from self.path_lines so that we
        # have the same number of GLLinePlotItems as paths
        if len(self.path_lines) < len(paths):
            num_lines_to_add = len(paths) - len(self.path_lines)

            for i in range(num_lines_to_add):
                gl_line_plot_item = GLLinePlotItem()
                self.path_lines.append(gl_line_plot_item)
                added_graphics.append(gl_line_plot_item)

        elif len(self.path_lines) > len(paths):
            num_lines_to_remove = len(self.path_lines) - len(paths)

            for i in range(num_lines_to_remove):
                removed_graphics.append(self.path_lines.pop())

        for index, path in enumerate(paths):
            path_line = self.path_lines[index]
            path_line.setData(
                pos=np.array([[point.x_meters, point.y_meters, 0] for point in path.points]),
                color=Colors.NAVIGATOR_PATH_COLOR
            )

        if len(self.desired_position_outlines) < len(requested_destinations):
            num_lines_to_add = len(requested_destinations) - len(self.desired_position_outlines)

            for i in range(num_lines_to_add):
                gl_line_plot_item = GLRobotOutline(color=Colors.DESIRED_ROBOT_LOCATION_OUTLINE)
                self.desired_position_outlines.append(gl_line_plot_item)
                added_graphics.append(gl_line_plot_item)

        elif len(self.desired_position_outlines) > len(requested_destinations):
            num_lines_to_remove = len(self.desired_position_outlines) - len(requested_destinations)

            for i in range(num_lines_to_remove):
                removed_graphics.append(self.desired_position_outlines.pop())

        for (dest, final_angle), outline in zip(
            requested_destinations, self.desired_position_outlines
        ):
            outline.setPosition(dest.x_meters, dest.y_meters)
            outline.setOrientation(final_angle.radians)

        return added_graphics, removed_graphics

        