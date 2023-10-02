from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl import *

import math
import numpy as np

from software.py_constants import *
from software.thunderscope.constants import Colors

from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.gl.graphics.gl_sphere import GLSphere
from software.thunderscope.gl.helpers.extended_gl_view_widget import PointInSceneEvent

from software.thunderscope.gl.helpers.observable_list import ObservableList


class GLMeasureLayer(GLLayer):
    """GLLayer that displays UI graphics for measuring coordinates, distances, and angles"""

    def __init__(self, name: str):
        """Initialize the GLMeasureLayer
        
        :param name: The displayed name of the layer

        """
        GLLayer.__init__(self, name)

        self.mouse_point_in_scene = [0, 0]
        self.measurement_points_cache = []

        # GLTextItem must be initialized later, outside of this constructor
        # Otherwise we run into some strange bugs: 'NoneType' object has no attribute 'width'
        self.cursor_coords_graphic: GLTextItem = None

        self.measurement_text_graphics = ObservableList(self._graphics_changed)
        self.measurement_line_graphics = ObservableList(self._graphics_changed)
        self.measurement_point_graphics = ObservableList(self._graphics_changed)

    def mouse_in_scene_pressed(self, event: PointInSceneEvent):
        """Event handler for the mouse_in_scene_pressed event
        
        :param event: The event

        """
        self.measurement_points_cache.append(event.point_in_scene)

        measurement_point_graphic = GLSphere(
            radius=0.02, color=Colors.PRIMARY_TEXT_COLOR
        )
        measurement_point_graphic.set_position(
            event.point_in_scene[0], event.point_in_scene[1], 0
        )
        self.measurement_point_graphics.append(measurement_point_graphic)

        # If we have at least one previous measurement point, then adding a new point
        # will create a line between the last point and the new point
        if len(self.measurement_points_cache) > 1:

            first_point = self.measurement_points_cache[-2]
            second_point = self.measurement_points_cache[-1]

            # Create and add line graphic
            measurement_line_graphic = GLLinePlotItem(
                color=(Colors.PRIMARY_TEXT_COLOR),
                pos=np.array(
                    [
                        [first_point[0], first_point[1], 0],
                        [second_point[0], second_point[1], 0],
                    ]
                ),
            )
            self.measurement_line_graphics.append(measurement_line_graphic)

            # Create and add text graphic labelling the line with the measurement

            # Pythagorean theorem
            distance = math.sqrt(
                (second_point[0] - first_point[0]) ** 2
                + (second_point[1] - first_point[1]) ** 2
            )

            midpoint = [
                (first_point[0] + second_point[0]) / 2,
                (first_point[1] + second_point[1]) / 2,
            ]

            self.measurement_text_graphics.append(
                GLTextItem(
                    font=QtGui.QFont("Roboto", 8),
                    color=Colors.PRIMARY_TEXT_COLOR,
                    text=f"{distance:.2f} m",
                    pos=np.array([*midpoint, 0]),
                )
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

            self.measurement_text_graphics.append(
                GLTextItem(
                    font=QtGui.QFont("Roboto", 8),
                    color=Colors.PRIMARY_TEXT_COLOR,
                    text=f"{angle:.1f}°",
                    pos=np.array([*placement_point, 0]),
                )
            )

            # Clear the point cache
            self.measurement_points_cache.clear()

    def mouse_in_scene_moved(self, event: PointInSceneEvent):
        """Event handler for the mouse_in_scene_moved event
        
        :param event: The event
        
        """
        self.mouse_point_in_scene = event.point_in_scene

    def clear_measurements(self):
        """Clear all measurements in the layer"""
        self.measurement_text_graphics.clear()
        self.measurement_line_graphics.clear()
        self.measurement_point_graphics.clear()
        self.measurement_points_cache.clear()

    def refresh_graphics(self):
        """Update graphics in this layer"""

        # Display coordinates of point at mouse cursor

        if not self.cursor_coords_graphic:
            self.cursor_coords_graphic = GLTextItem(
                parentItem=self,
                font=QtGui.QFont("Roboto", 10),
                color=Colors.PRIMARY_TEXT_COLOR,
            )

        mouse_point_in_scene_x = self.mouse_point_in_scene[0]
        mouse_point_in_scene_y = self.mouse_point_in_scene[1]
        self.cursor_coords_graphic.setData(
            text=f"({mouse_point_in_scene_x:.2f} m, {mouse_point_in_scene_y:.2f} m)",
            pos=[mouse_point_in_scene_x, mouse_point_in_scene_y, 0],
        )
