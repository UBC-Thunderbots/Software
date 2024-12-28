from software.thunderscope.binary_context_managers.full_system import ProtoUnixIO
from software.thunderscope.gl.graphics.gl_polygon import GLPolygon
from pyqtgraph.Qt import QtGui
from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.opengl import *

from proto.import_all_protos import *
from software.py_constants import *
from software.thunderscope.gl.helpers.observable_list import ObservableList
from software.thunderscope.proto_unix_io import ProtoUnixIO

from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.gl.helpers.extended_gl_view_widget import MouseInSceneEvent


class GLDrawPolygonObstacleLayer(GLLayer):
    """A layer used to draw polygons that are going to represent obstacles for the trajectory planner
    to avoid.
    """

    def __init__(self, name: str, friendly_io: ProtoUnixIO) -> None:
        """Initialize this layer

        :param name:            the name of this layer
        :param friendly_io:     the friendly_io
        """
        super().__init__(name)

        self.friendly_io: ProtoUnixIO = friendly_io

        self.current_polygon: GLPolygon = GLPolygon(parent_item=self, line_width=2)
        # Tuple[float, float] represents a point (x,y)
        self.points: List[Tuple[float, float]] = []

        self.obstacles: List[Obstacle] = []

        # used for keeping track and rendering multiple polygons
        self.rendering_polygons: ObservableList = ObservableList(self._graphics_changed)

    def keyPressEvent(self, event: QtGui.QKeyEvent) -> None:
        """Responding to key events that are going to push obstacles to the stack or add point

        :param event: a qt key event indicating the button that are pressed
        """
        if not self.visible():
            return

        if event.key() == Qt.Key.Key_P:
            self.push_polygon_to_list()

        if event.key() == Qt.Key.Key_C:
            self.clear_polygons()

    def clear_polygons(self):
        """Clearing the obstacles"""
        self.points.clear()
        self.obstacles.clear()

        for polygon in self.rendering_polygons:
            polygon.hide()
        self.rendering_polygons.resize(0, lambda: {})
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

    def mouse_in_scene_pressed(self, event: MouseInSceneEvent) -> None:
        """Adding the point in scene

        :param event: the mouse event
        """
        if not self.visible():
            return

        point = event.point_in_scene
        self._add_one_point((point.x(), point.y()))

        return super().mouse_in_scene_pressed(event)
