import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.opengl import *

import numpy as np


class PointInSceneEvent(object):
    """Wraps QMouseEvent and includes additional data about the point in the 3D scene
    that was picked by the mouse cursor
    """

    def __init__(self, mouse_event: QtGui.QMouseEvent, point_in_scene):
        """Initialize the PointInSceneEvent
        
        :param mouse_event: The QMouseEvent to wrap
        :param point_in_scene: The point in the 3D scene that was picked
        
        """
        self.mouse_event = mouse_event
        self.point_in_scene = point_in_scene


class ExtendedGLViewWidget(GLViewWidget):
    """Extends GLViewWidget with ability to determine coordinates of the
    mouse cursor position in the 3D scene"""

    # Signal emitted when mouse has picked a point in the 3D scene (shift + click)
    point_in_scene_pressed_signal = QtCore.pyqtSignal(PointInSceneEvent)

    # Signal emitted when mouse is dragging within the 3D scene (shift + drag)
    point_in_scene_dragged_signal = QtCore.pyqtSignal(PointInSceneEvent)

    # Signal emitted when mouse is released having picked a point in the 3D scene (shift + release)
    point_in_scene_released_signal = QtCore.pyqtSignal(PointInSceneEvent)

    def __init__(self):
        """Initialize the ExtendedGLViewWidget"""
        super().__init__()

        # Keep track of whether the mouse picked a point in the 3D scene
        self.point_picked = False

    def mousePressEvent(self, event):
        """Detect that the mouse was pressed
        
        :param event: The event
        
        """
        if (
            event.buttons() == QtCore.Qt.MouseButton.LeftButton
            and event.modifiers() == QtCore.Qt.KeyboardModifier.ShiftModifier
        ):
            self.point_picked = True
            point_in_scene_event = PointInSceneEvent(
                event, self.get_point_in_scene(event.position())
            )
            self.point_in_scene_pressed_signal.emit(point_in_scene_event)
        else:
            # Only handle GLViewWidget orbit/pan if we're not picking a point in 3D
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        """Detect that the mouse was moved
        
        :param event: The event

        """
        if self.point_picked:
            point_in_scene_event = PointInSceneEvent(
                event, self.get_point_in_scene(event.position())
            )
            self.point_in_scene_dragged_signal.emit(point_in_scene_event)
        else:
            # Only handle GLViewWidget orbit/pan if we're not picking a point in 3D
            super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        """Detect that the mouse was released
        
        :param event: The event

        """
        if self.point_picked:
            self.point_picked = False
            point_in_scene_event = PointInSceneEvent(
                event, self.get_point_in_scene(event.position())
            )
            self.point_in_scene_released_signal.emit(point_in_scene_event)
        else:
            # Only handle GLViewWidget orbit/pan if we're not picking a point in 3D
            super().mouseReleaseEvent(event)

    def get_point_in_scene(self, mouse_pos):
        """Determine the coordinates of the point on the x-y plane in the 3D scene that 
        the mouse is pointing at.

        :param mouse_pos: the coordinates of the mouse relative to the ExtendedGLViewWidget
        :returns: the point in the 3D scene that represents where the mouse is pointing to

        """
        # Get viewport
        viewport_width = self.width()
        viewport_height = self.height()
        viewport = QtCore.QRect(0, 0, viewport_width, viewport_height)

        # Window space coordinates on far plane (z = 1)
        # Y coordinate is "inverted" since OpenGL convention expects bottom of viewport
        # to be 0, whereas for Qt top is 0
        window_space_coords = QtGui.QVector3D(
            mouse_pos.x(), viewport_height - mouse_pos.y(), 1
        )

        # Get projection and view matrices
        projection_matrix = self.projectionMatrix()
        view_matrix = self.viewMatrix()

        # Unproject point on far plane to get get ray direction vector
        ray_dir = window_space_coords.unproject(
            view_matrix, projection_matrix, viewport
        )

        # Ray origin is the camera position
        ray_origin = self.cameraPosition()

        # Point and normal vector of x-y plane
        plane_point = QtGui.QVector3D(0, 0, 0)
        plane_normal = QtGui.QVector3D(0, 0, 1)

        # Find intersection of ray with plane
        distance_from_ray_origin = (
            QtGui.QVector3D.dotProduct(plane_point, plane_normal)
            - QtGui.QVector3D.dotProduct(plane_normal, ray_origin)
        ) / QtGui.QVector3D.dotProduct(plane_normal, ray_dir)
        intersection = ray_origin + (distance_from_ray_origin * ray_dir)

        return intersection
