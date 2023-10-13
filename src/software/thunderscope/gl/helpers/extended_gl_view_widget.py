import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.opengl import *
from software.py_constants import ROBOT_MAX_HEIGHT_METERS
from software.thunderscope.constants import MULTI_PLANE_POINTS

import numpy as np
from typing import List


class MouseInSceneEvent:
    """Wraps QMouseEvent and includes additional data about the point in the 3D scene
    that was picked by the mouse cursor
    as well as points in other planes that correspond to the mouse cursor
    """

    def __init__(
        self,
        mouse_event: QtGui.QMouseEvent,
        point_in_scene: QtGui.QVector3D,
        multi_plane_points: List[QtGui.QVector3D],
    ):
        """Initialize the MouseInSceneEvent

        :param mouse_event: The QMouseEvent to wrap
        :param point_in_scene: The main point in the 3D scene that was picked
        :param multi_plane_points: Points on multiple planes that correspond to the mouse event
        """
        self.mouse_event = mouse_event
        self.point_in_scene = point_in_scene
        self.multi_plane_points = multi_plane_points


class ExtendedGLViewWidget(GLViewWidget):
    """Extends GLViewWidget with ability to determine coordinates of the
    mouse cursor position in the 3D scene"""

    # Signal emitted when mouse has picked a point in the 3D scene (shift + click)
    mouse_in_scene_pressed_signal = QtCore.pyqtSignal(MouseInSceneEvent)

    # Signal emitted when mouse is dragging within the 3D scene (shift + drag)
    mouse_in_scene_dragged_signal = QtCore.pyqtSignal(MouseInSceneEvent)

    # Signal emitted when mouse is released after picked a point in the 3D scene (shift + release)
    mouse_in_scene_released_signal = QtCore.pyqtSignal(MouseInSceneEvent)

    # Signal emitted when mouse is moving within the 3D scene
    # (detect_mouse_movement_in_scene must be enabled for this signal to be emitted)
    mouse_in_scene_moved_signal = QtCore.pyqtSignal(MouseInSceneEvent)

    def __init__(self):
        """Initialize the ExtendedGLViewWidget"""
        super().__init__()

        # Fixes strange bug where mousePos is not initialized
        self.mousePos = QtCore.QPointF(0, 0)

        # Always track mouse so that mouseMoveEvent is always received
        self.setMouseTracking(True)

        # Keep track of whether the mouse picked a point in the 3D scene
        self.point_picked = False

        # This must be enabled for the mouse_moved_in_scene_signal to be emitted
        self.detect_mouse_movement_in_scene = False

    def mousePressEvent(self, event: QtGui.QMouseEvent):
        """Detect that the mouse was pressed

        If Shift is pressed along with mouse click, calculate which point was picked in scene
        Forward this point using the mouse pressed signal

        If Shift is not pressed, let super method handle it

        :param event: The event
        """
        if (
            event.buttons() == QtCore.Qt.MouseButton.LeftButton
            and event.modifiers() & QtCore.Qt.KeyboardModifier.ShiftModifier
        ):
            self.point_picked = True
            point_in_scene_event = MouseInSceneEvent(
                event,
                self.get_point_in_scene(event.position()),
                self.get_multi_plane_points_in_scene(event.position()),
            )
            self.mouse_in_scene_pressed_signal.emit(point_in_scene_event)
        else:
            # Only handle GLViewWidget orbit/pan if we're not picking a point in 3D
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QtGui.QMouseEvent):
        """Detect that the mouse was moved
        
        :param event: The event

        """
        if self.point_picked or self.detect_mouse_movement_in_scene:

            point_in_scene_event = MouseInSceneEvent(
                event,
                self.get_point_in_scene(event.position()),
                self.get_multi_plane_points_in_scene(event.position()),
            )

            if self.detect_mouse_movement_in_scene:
                self.mouse_in_scene_moved_signal.emit(point_in_scene_event)

            if self.point_picked:
                self.mouse_in_scene_dragged_signal.emit(point_in_scene_event)
                # We don't want to handle GLViewWidget orbit/pan if we're picking a
                # point in 3D, so return early
                return

        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QtGui.QMouseEvent):
        """Detect that the mouse was released
        
        :param event: The event

        """
        if self.point_picked:
            self.point_picked = False
            point_in_scene_event = MouseInSceneEvent(
                event,
                self.get_point_in_scene(event.position()),
                self.get_multi_plane_points_in_scene(event.position()),
            )
            self.mouse_in_scene_released_signal.emit(point_in_scene_event)
        else:
            # Only handle GLViewWidget orbit/pan if we're not picking a point in 3D
            super().mouseReleaseEvent(event)

    def get_multi_plane_points_in_scene(
        self, mouse_pos: QtCore.QPoint
    ) -> List[QtGui.QVector3D]:
        """
        Determines the coordinates of the points on the x-y planes and a few planes above it
        in the 3D scene that the mouse is pointing at
        :param mouse_pos: the coordinates of the mouse relative to the ExtendedGLViewWidget
        :return: a list of points in the 3D scene representing where the mouse is pointing to on multiple planes
        """
        return [
            self.get_point_in_scene(
                mouse_pos,
                ROBOT_MAX_HEIGHT_METERS * (float(z_div) / (MULTI_PLANE_POINTS - 1)),
            )
            for z_div in range(0, MULTI_PLANE_POINTS)
        ]

    def get_point_in_scene(
        self, mouse_pos: QtCore.QPoint, z_height: float = 0
    ) -> QtGui.QVector3D:
        """Determine the coordinates of the point on the x-y plane in the 3D scene that
        the mouse is pointing at.

        :param mouse_pos: the coordinates of the mouse relative to the ExtendedGLViewWidget
        :param z_height: the z axis height of the plane that the mouse event should point to
                            default is 0 (x-y plane)
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

        # Unproject point on far plane to get the ray direction vector
        ray_dir = window_space_coords.unproject(
            view_matrix, projection_matrix, viewport
        )

        # Ray origin is the camera position
        ray_origin = self.cameraPosition()

        # Point and normal vector of x-y plane
        plane_point = QtGui.QVector3D(0, 0, z_height)
        plane_normal = QtGui.QVector3D(0, 0, 1)

        # Find intersection of ray with plane
        distance_from_ray_origin = (
            QtGui.QVector3D.dotProduct(plane_point, plane_normal)
            - QtGui.QVector3D.dotProduct(plane_normal, ray_origin)
        ) / QtGui.QVector3D.dotProduct(plane_normal, ray_dir)
        intersection = ray_origin + (distance_from_ray_origin * ray_dir)

        return intersection
