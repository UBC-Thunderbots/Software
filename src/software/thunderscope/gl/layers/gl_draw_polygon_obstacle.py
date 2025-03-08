from software.thunderscope.binary_context_managers.full_system import ProtoUnixIO
from software.thunderscope.gl.graphics.gl_polygon import GLPolygon
from pyqtgraph.Qt import QtGui
from pyqtgraph.Qt.QtCore import QTimer, Qt
from pyqtgraph.opengl import *

from proto.import_all_protos import *
from software.py_constants import *
from software.thunderscope.gl.helpers.observable_list import ObservableList
from software.thunderscope.proto_unix_io import ProtoUnixIO

from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.gl.helpers.extended_gl_view_widget import MouseInSceneEvent


class GLDrawPolygonObstacleLayer(GLLayer):
    """A layer used to draw polygons representing virtual obstacles for the 
    trajectory planner to avoid.
    """

    DOUBLE_CLICK_INTERVAL = 200

    def __init__(self, name: str, friendly_io: ProtoUnixIO) -> None:
        """Initialize this layer

        :param name:            the name of this layer
        :param friendly_io:     the friendly_io
        """
        super().__init__(name)

        self.friendly_io = friendly_io

        self.current_polygon = GLPolygon(parent_item=self, line_width=2)
        self.points: List[Tuple[float, float]] = []

        self.obstacles: List[Obstacle] = []

        # used for keeping track and rendering multiple polygons
        self.rendering_polygons = ObservableList(self._graphics_changed)

        self.can_double_click = True

    def keyPressEvent(self, event: QtGui.QKeyEvent) -> None:
        """Responding to key events that are going to push obstacles to the stack or add point

        :param event: a qt key event indicating the button that are pressed
        """
        if not self.visible():
            return

        if event.key() == Qt.Key.Key_C:
            self.clear_polygons()

    def clear_polygons(self):
        """Clearing the obstacles"""
        self.points.clear()
        self.obstacles.clear()

        for polygon in self.rendering_polygons:
            polygon.hide()
        self.rendering_polygons.clear()
        self.current_polygon.hide()
        self.current_polygon = GLPolygon(parent_item=self, line_width=2)

        self._send_to_fs()

    def push_polygon_to_list(self):
        """Pushing the fully drawn polygon to the stack"""
        points = [
            Point(x_meters=point[0], y_meters=point[1]) for point in self.points[:-1]
        ]

        if len(points) <= 2:
            print("Cannot push polygon to stack as there are less than two points.")
            return

        polygon = Polygon(points=points)
        obstacle = Obstacle(polygon=polygon)
        self.obstacles.append(obstacle)
        self.points.clear()

        self.rendering_polygons.append(self.current_polygon)
        self.current_polygon = GLPolygon(parent_item=self, line_width=2)

    def _add_one_point(self, point: tuple[float, float]):
        """Adding one points to a polygon

        :param point: represent the point (x,y) that is added to the polygon
        """
        if len(self.points) < 2:
            # creating a line segment
            self.points.append(point)
        elif len(self.points) == 2:
            # creating a triangle
            start_point = self.points[0]
            self.points.append(point)
            self.points.append(start_point)
        else:
            # creating a general polygon
            start_point = self.points[0]
            self.points.pop()  # removing the start point since the last point is always the start point

            self.points.append(point)
            self.points.append(start_point)
        self.current_polygon.set_points(self.points)
        self._send_to_fs()

    def _send_to_fs(self):
        """Sending a list of virtual obstacles to full system"""
        obstacles = self.obstacles.copy()

        points = [
            Point(x_meters=point[0], y_meters=point[1]) for point in self.points[:-1]
        ]

        # only send to full system when the points form a valid polygon
        if len(points) >= 3:
            polygon = Polygon(points=points)
            obstacle = Obstacle(polygon=polygon)
            obstacles.append(obstacle)

        self.friendly_io.send_proto(
            VirtualObstacles, VirtualObstacles(obstacles=obstacles)
        )

    def create_single_click_callback(self, event: MouseInSceneEvent):
        """Creating a single shot callback to handle single click

        :param event: The mouse event when a scene is pressed
        """

        def _handle_single_click():
            # This logic is somewhat non trivial. If we `can_double_click`, it indicates that
            # a double-click hasn't occurred within the 200 ms time window after the first click.
            # In other words, the user hasn't double-clicked, so we will now interpret the action 
            # as a single click.
            if self.can_double_click:
                point = event.point_in_scene
                self._add_one_point((point.x(), point.y()))

            self.can_double_click = False

        return _handle_single_click

    def mouse_in_scene_pressed(self, event: MouseInSceneEvent) -> None:
        """Adding the point in scene

        :param event: the mouse event
        """
        if not self.visible():
            return
        
        if not event.mouse_event.modifiers() == Qt.KeyboardModifier.ShiftModifier | Qt.KeyboardModifier.AltModifier:
            return

        # handle double click
        if self.can_double_click:
            self.push_polygon_to_list()
            self.can_double_click = False
        else:
            self.can_double_click = True
            # handle single click
            QTimer.singleShot(
                self.DOUBLE_CLICK_INTERVAL, self.create_single_click_callback(event)
            )

    def refresh_graphics(self) -> None:
        """Refreshing graphics"""
        return
