from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.opengl import *

import math
import numpy as np

from proto.geometry_pb2 import Circle, Polygon
from proto.tbots_software_msgs_pb2 import PrimitiveSet

import software.thunderscope.constants as constants
from software.py_constants import *
from software.thunderscope.constants import Colors
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.gl.gl_layer import GLLayer
from software.thunderscope.gl.graphics.gl_sphere import GLSphere
from software.thunderscope.gl.helpers.extended_gl_view_widget import PointInSceneEvent


class GLMeasureLayer(GLLayer):
    """GLLayer that displays UI graphics for measuring coordinates, distances, and angles"""

    def __init__(self, name: str):
        """Initialize the GLMeasureLayer
        
        :param name: The displayed name of the layer

        """
        GLLayer.__init__(self, name)

        self.mouse_point_in_scene = [0, 0]

        self.measurement_points = []
        self.measurement_lines = []
        self.measurement_angles = []

        self.measurement_points_cache = []

        self.graphics_list.register_graphics_group(
            "cursor_text",
            lambda: GLTextItem(
                font=QtGui.QFont("Roboto", 10), color=Colors.PRIMARY_TEXT_COLOR
            ),
        )
        self.graphics_list.register_graphics_group(
            "measurement_text",
            lambda: GLTextItem(
                font=QtGui.QFont("Roboto", 8), color=Colors.PRIMARY_TEXT_COLOR
            ),
        )
        self.graphics_list.register_graphics_group(
            "measurement_lines", lambda: GLLinePlotItem(color=Colors.PRIMARY_TEXT_COLOR)
        )
        self.graphics_list.register_graphics_group(
            "measurement_points",
            lambda: GLSphere(radius=0.02, color=Colors.PRIMARY_TEXT_COLOR,),
        )

    def mouse_in_scene_pressed(self, event: PointInSceneEvent):
        """Event handler for the mouse_in_scene_pressed event
        
        :param event: The event

        """
        self.measurement_points_cache.append(event.point_in_scene)
        self.measurement_points.append(event.point_in_scene)

        # If we have at least one previous measurement point, then adding a new point
        # will create a line between the last point and the new point
        if len(self.measurement_points_cache) > 1:
            self.measurement_lines.append(
                [self.measurement_points_cache[-2], self.measurement_points_cache[-1]]
            )

        # If two points are already in the cache, adding a new point will form an angle
        # between the three points
        if len(self.measurement_points_cache) == 3:

            # Calculate the angle
            a = self.measurement_points_cache[0]
            b = self.measurement_points_cache[1]
            c = self.measurement_points_cache[2]

            ba = a - b
            bc = c - b

            cosine_angle = QtGui.QVector3D.dotProduct(ba, bc) / (
                ba.length() * bc.length()
            )
            angle = math.degrees(math.acos(cosine_angle))

            # Calculate the point where the angle text should be placed.
            # This point should be a constant distance along the angle bisector, starting
            # from the angle vertex
            bisector = ba.normalized() + bc.normalized()
            placement_point = b + 0.5 * bisector.normalized()

            self.measurement_angles.append(
                [angle, [placement_point[0], placement_point[1], 0]]
            )

            # Clear the cache
            self.measurement_points_cache.clear()

    def mouse_in_scene_moved(self, event: PointInSceneEvent):
        """Event handler for the mouse_in_scene_moved event
        
        :param event: The event
        
        """
        self.mouse_point_in_scene = event.point_in_scene

    def clear_measurements(self):
        """Clear all measurements in the layer"""
        self.measurement_points.clear()
        self.measurement_lines.clear()
        self.measurement_angles.clear()
        self.measurement_points_cache.clear()

    def _update_graphics(self):
        """Fetch and update graphics for the layer"""

        # Display coordinates of point at mouse cursor
        cursor_text_graphic = self.graphics_list.get_graphics("cursor_text", 1)[0]
        mouse_point_in_scene_x = self.mouse_point_in_scene[0]
        mouse_point_in_scene_y = self.mouse_point_in_scene[1]
        cursor_text_graphic.setData(
            text=f"({mouse_point_in_scene_x:.2f} m, {mouse_point_in_scene_y:.2f} m)",
            pos=[mouse_point_in_scene_x, mouse_point_in_scene_y, 0],
        )

        self.__update_measurement_graphics()

    def __update_measurement_graphics(self):
        """Update the graphics that display the measurements"""

        for measurement_point_graphic, measurement_point in zip(
            self.graphics_list.get_graphics(
                "measurement_points", len(self.measurement_points)
            ),
            self.measurement_points,
        ):
            measurement_point_graphic.set_position(
                measurement_point[0], measurement_point[1], 0
            )

        for measurement_text_graphic, measurement_line_graphic, measurement_line in zip(
            self.graphics_list.get_graphics(
                "measurement_text", len(self.measurement_lines)
            ),
            self.graphics_list.get_graphics(
                "measurement_lines", len(self.measurement_lines)
            ),
            self.measurement_lines,
        ):
            first_point = measurement_line[0]
            second_point = measurement_line[1]

            # Pythagorean theorem
            distance = math.sqrt(
                (second_point[0] - first_point[0]) ** 2
                + (second_point[1] - first_point[1]) ** 2
            )

            midpoint = [
                (first_point[0] + second_point[0]) / 2,
                (first_point[1] + second_point[1]) / 2,
            ]

            measurement_text_graphic.setData(
                text=f"{distance:.2f} m", pos=np.array([midpoint[0], midpoint[1], 0]),
            )

            measurement_line_graphic.setData(
                pos=np.array(
                    [
                        [first_point[0], first_point[1], 0],
                        [second_point[0], second_point[1], 0],
                    ]
                ),
            )

        for measurement_angle_graphic, measurement_angle in zip(
            self.graphics_list.get_graphics(
                "measurement_text", len(self.measurement_angles)
            ),
            self.measurement_angles,
        ):
            angle = measurement_angle[0]
            placement_point = measurement_angle[1]

            measurement_angle_graphic.setData(
                text=f"{angle:.1f}°", pos=np.array(placement_point),
            )
