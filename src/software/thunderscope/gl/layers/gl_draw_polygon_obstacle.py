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

from typing import Callable, override


class GLDrawPolygonObstacleLayer(GLLayer):
    """A layer used to draw polygons representing virtual obstacles for the
    trajectory planner to avoid.
    """

    DOUBLE_CLICK_INTERVAL_MS = 200

    def __init__(self, name: str, friendly_io: ProtoUnixIO) -> None:
        """Initialize this layer

        :param name:            the name of this layer
        :param friendly_io:     the friendly_io
        """
        super().__init__(name)

        self.friendly_io = friendly_io

        self.points: List[Point] = []
        self.obstacles: List[Obstacle] = []

        # Stores the polygons that are currently visible
        self.rendering_polygons = ObservableList(self._graphics_changed)

        # The current polygon being edited (not visible yet)
        self.current_polygon = GLPolygon(parent_item=self, line_width=2, closed=True)

        self.can_double_click = False

    @override
    def keyPressEvent(self, event: QtGui.QKeyEvent) -> None:
        """Responding to key events that are going to push obstacles to the stack or add point

        :param event: a qt key event indicating the button that are pressed
        """
        if not self.visible():
            return

        if event.key() == Qt.Key.Key_C:
            self.__clear_polygons()

    def __clear_polygons(self) -> None:
        """Clearing the obstacles"""
        self.points.clear()
        self.obstacles.clear()

        self.rendering_polygons.clear()
        self.current_polygon.setParentItem(None)

        self.__send_to_fullsystem()

    def __push_polygon_to_list(self):
        """Pushing the fully drawn polygon to the stack"""
        if len(self.points) <= 2:
            print("Cannot push polygon to stack as there are less than two points.")
            return

        self.obstacles.append(Obstacle(polygon=Polygon(points=self.points.copy())))
        self.points.clear()

        self.rendering_polygons.append(self.current_polygon)
        self.current_polygon = GLPolygon(parent_item=self, line_width=2, closed=True)

    def __add_one_point(self, point: Point) -> None:
        """Adding one points to a polygon

        :param point: represent the point (x,y) that is added to the polygon
        """
        self.points.append(point)
        self.current_polygon.set_points(
            [(point.x_meters, point.y_meters) for point in self.points]
        )
        self.__send_to_fullsystem()

    def __send_to_fullsystem(self) -> None:
        """Sending a list of virtual obstacles to full system"""
        # only send to full system when the points form a valid polygon
        if len(self.points) >= 3:
            obstacles = self.obstacles.copy()

            polygon = Polygon(points=self.points.copy())
            obstacle = Obstacle(polygon=polygon)
            obstacles.append(obstacle)

            self.friendly_io.send_proto(
                VirtualObstacles, VirtualObstacles(obstacles=obstacles)
            )

    def __create_single_click_callback(
        self, event: MouseInSceneEvent
    ) -> Callable[[], None]:
        """Creating a single shot callback to handle single click

        :param event: The mouse event when a scene is pressed
        :return: callback that handles a single click
        """

        def __handle_single_click():
            # This logic is somewhat non trivial. If we `can_double_click`, it indicates that
            # a double-click hasn't occurred within the 200 ms time window after the first click.
            # In other words, the user hasn't double-clicked, so we will now interpret the action
            # as a single click.
            if self.can_double_click:
                point = event.point_in_scene
                self.__add_one_point(Point(x_meters=point.x(), y_meters=point.y()))

            self.can_double_click = False

        return __handle_single_click

    @override
    def mouse_in_scene_pressed(self, event: MouseInSceneEvent) -> None:
        """Adding the point in scene

        :param event: the mouse event
        """
        if not self.visible():
            return

        if (
            not event.mouse_event.modifiers()
            == Qt.KeyboardModifier.ShiftModifier | Qt.KeyboardModifier.AltModifier
        ):
            return

        # handle double click
        if self.can_double_click:
            self.__push_polygon_to_list()
            self.can_double_click = False
        else:
            self.can_double_click = True
            # handle single click
            QTimer.singleShot(
                self.DOUBLE_CLICK_INTERVAL_MS,
                self.__create_single_click_callback(event),
            )

    @override
    def refresh_graphics(self) -> None:
        """Refreshing graphics"""
        return
