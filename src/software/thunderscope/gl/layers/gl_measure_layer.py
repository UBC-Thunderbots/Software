from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl import *

import math
import numpy as np

from software.py_constants import *
from software.thunderscope.constants import Colors, LINE_WIDTH

from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.gl.graphics.gl_sphere import GLSphere
from software.thunderscope.gl.graphics.gl_polygon import GLPolygon
from software.thunderscope.gl.helpers.extended_gl_view_widget import MouseInSceneEvent

from software.thunderscope.gl.helpers.observable_list import ObservableList


class GLMeasureLayer(GLLayer):
    """GLLayer that displays UI graphics for measuring coordinates, distances, and angles"""

    def __init__(self, name: str) -> None:
        """Initialize the GLMeasureLayer
        
        :param name: The displayed name of the layer

        """
        super().__init__(name)

        self.mouse_point_in_scene = QtGui.QVector3D()
        self.measurement_points_cache = []

        # GLTextItem must be initialized later, outside of this constructor
        # Otherwise we run into some strange bugs: 'NoneType' object has no attribute 'width'
        self.cursor_coords_graphic: GLTextItem = None

        self.measurement_text_graphics = ObservableList(self._graphics_changed)
        self.measurement_line_graphics = ObservableList(self._graphics_changed)
        self.measurement_point_graphics = ObservableList(self._graphics_changed)

    def mouse_in_scene_pressed(self, event: MouseInSceneEvent) -> None:
        """Detect that the mouse was pressed and picked a point in the 3D scene
        
        :param event: The event

        """
        self.measurement_points_cache.append(event.point_in_scene)

        measurement_point_graphic = GLSphere(
            radius=0.02, color=Colors.PRIMARY_TEXT_COLOR
        )
        measurement_point_graphic.set_position(
            event.point_in_scene.x(), event.point_in_scene.y(), 0
        )
        self.measurement_point_graphics.append(measurement_point_graphic)

        # If we have at least one previous measurement point, then adding a new point
        # will create a line between the last point and the new point
        if len(self.measurement_points_cache) > 1:

            first_point = self.measurement_points_cache[-2]
            second_point = self.measurement_points_cache[-1]

            # Create and add line graphic
            self.measurement_line_graphics.append(
                GLPolygon(
                    outline_color=(Colors.PRIMARY_TEXT_COLOR),
                    line_width=LINE_WIDTH,
                    points=[
                        [first_point.x(), first_point.y(), 0],
                        [second_point.x(), second_point.y(), 0],
                    ],
                )
            )

            # Create and add text graphic labelling the line with the measurement

            # Pythagorean theorem
            distance = math.sqrt(
                (second_point.x() - first_point.x()) ** 2
                + (second_point.y() - first_point.y()) ** 2
            )
            midpoint = (first_point + second_point) / 2

            self.measurement_text_graphics.append(
                GLTextItem(
                    font=QtGui.QFont("Roboto", 8),
                    color=Colors.PRIMARY_TEXT_COLOR,
                    text=f"{distance:.2f} m",
                    pos=np.array([midpoint.x(), midpoint.y(), 0]),
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
                    pos=np.array([placement_point.x(), placement_point.y(), 0]),
                )
            )

            # Clear the point cache
            self.measurement_points_cache.clear()

    def mouse_in_scene_moved(self, event: MouseInSceneEvent) -> None:
        """Detect that the mouse was moved within the 3D scene
        
        :param event: The event
        
        """
        self.mouse_point_in_scene = event.point_in_scene

    def clear_measurements(self) -> None:
        """Clear all measurements in the layer"""
        self.measurement_text_graphics.clear()
        self.measurement_line_graphics.clear()
        self.measurement_point_graphics.clear()
        self.measurement_points_cache.clear()

    def refresh_graphics(self) -> None:
        """Update graphics in this layer"""

        # Display coordinates of point at mouse cursor

        if not self.cursor_coords_graphic:
            self.cursor_coords_graphic = GLTextItem(
                parentItem=self,
                font=QtGui.QFont("Roboto", 10),
                color=Colors.PRIMARY_TEXT_COLOR,
            )

        self.cursor_coords_graphic.setData(
            text=f"({self.mouse_point_in_scene.x():.2f} m, {self.mouse_point_in_scene.y():.2f} m)",
            pos=[self.mouse_point_in_scene.x(), self.mouse_point_in_scene.y(), 0],
        )
