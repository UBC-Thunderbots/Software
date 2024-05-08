import logging
import time

from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl import *

from proto.visualization_pb2 import DebugShapes
from proto.world_pb2 import SimulationState

from software.thunderscope.constants import Colors, DepthValues
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.gl.graphics.gl_circle import GLCircle
from software.thunderscope.gl.graphics.gl_polygon import GLPolygon
from software.thunderscope.gl.graphics.gl_stadium import GLStadium
from software.thunderscope.gl.helpers.observable_list import ObservableList


class GLDebugShapesLayer(GLLayer):
    """GLLayer that visualizes shapes"""

    MAX_GRAPHICS_DURATION_SEC = 0.5

    def __init__(self, name: str, buffer_size: int = 1) -> None:
        """Initialize the GLDebugShapeLayer

        :param name: The displayed name of the layer
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        super().__init__(name)
        self.setDepthValue(DepthValues.BACKGROUND_DEPTH)

        self.debug_shapes_buffer = ThreadSafeBuffer(buffer_size, DebugShapes)
        self.debug_shape_map = {}

        self.poly_shape_graphics = ObservableList(self._graphics_changed)
        self.poly_shape_name_graphics = ObservableList(self._graphics_changed)

        self.circle_shape_graphics = ObservableList(self._graphics_changed)
        self.circle_shape_name_graphics = ObservableList(self._graphics_changed)

        self.stadium_shape_graphics = ObservableList(self._graphics_changed)
        self.stadium_shape_name_graphics = ObservableList(self._graphics_changed)

    def refresh_graphics(self) -> None:
        """Update graphics in this layer"""
        # Add all new shapes to the map
        debug_shapes_proto = self.debug_shapes_buffer.get(block=False, return_cached=False)
        now = time.time()
        while debug_shapes_proto is not None:
            for debug_shape in debug_shapes_proto.debug_shapes:
                self.debug_shape_map[debug_shape.unique_id] = (debug_shape, now)

            debug_shapes_proto = self.debug_shapes_buffer.get(
                block=False, return_cached=False
            )

        # Remove all shapes that have not been updated recently
        poly_named_shapes = []
        circle_named_shapes = []
        stadium_named_shapes = []
        for unique_id, (debug_shape, last_updated) in list(self.debug_shape_map.items()):
            if now - last_updated > self.MAX_GRAPHICS_DURATION_SEC:
                del self.debug_shape_map[unique_id]
                continue

            shape = debug_shape.shape
            name = debug_shape.debug_text
            if shape.HasField("polygon"):
                poly_named_shapes.append((name, shape.polygon))
            elif shape.HasField("stadium"):
                stadium_named_shapes.append((name, shape.stadium))
            elif shape.HasField("circle"):
                circle_named_shapes.append((name, shape.circle))
            else:
                logging.warning(
                    f"{shape}s are not supported in the debug shapes layer!"
                )

        # Ensure we have the same number of graphics as shapes
        self.poly_shape_graphics.resize(
            len(poly_named_shapes),
            lambda: GLPolygon(outline_color=Colors.DEBUG_SHAPES_COLOR),
        )
        self.poly_shape_name_graphics.resize(
            len(poly_named_shapes),
            lambda: GLTextItem(
                font=QtGui.QFont("Roboto", 8), color=Colors.DEBUG_SHAPES_COLOR
            ),
        )

        self.circle_shape_graphics.resize(
            len(circle_named_shapes),
            lambda: GLCircle(outline_color=Colors.DEBUG_SHAPES_COLOR),
        )
        self.circle_shape_name_graphics.resize(
            len(circle_named_shapes),
            lambda: GLTextItem(
                font=QtGui.QFont("Roboto", 8), color=Colors.DEBUG_SHAPES_COLOR
            ),
        )

        self.stadium_shape_graphics.resize(
            len(stadium_named_shapes),
            lambda: GLStadium(outline_color=Colors.DEBUG_SHAPES_COLOR),
        )
        self.stadium_shape_name_graphics.resize(
            len(stadium_named_shapes),
            lambda: GLTextItem(
                font=QtGui.QFont("Roboto", 8), color=Colors.DEBUG_SHAPES_COLOR
            ),
        )

        # Update graphics with the new shapes and names
        for poly_shape_graphic, poly_shape_text_graphic, (name, poly_shape) in zip(
            self.poly_shape_graphics, self.poly_shape_name_graphics, poly_named_shapes
        ):
            # In order to close the polygon, we need to include the first point at the end of
            # the list of points in the polygon
            polygon_points = list(poly_shape.points) + poly_shape.points[:1]
            poly_shape_graphic.set_points(
                [[point.x_meters, point.y_meters] for point in polygon_points]
            )
            poly_shape_text_graphic.setData(
                text=name,
                pos=[
                    min(p.x_meters for p in poly_shape.points),
                    min(p.y_meters for p in poly_shape.points) - 0.1,
                    0,
                ],
            )

        for (
            circle_shape_graphic,
            circle_shape_text_graphic,
            (name, circle_shape),
        ) in zip(
            self.circle_shape_graphics,
            self.circle_shape_name_graphics,
            circle_named_shapes,
        ):
            circle_shape_graphic.set_radius(circle_shape.radius)
            circle_shape_graphic.set_position(
                circle_shape.origin.x_meters, circle_shape.origin.y_meters,
            )
            circle_shape_text_graphic.setData(
                text=name,
                pos=[
                    circle_shape.origin.x_meters,
                    circle_shape.origin.y_meters - circle_shape.radius - 0.1,
                    0,
                ],
            )

        for (
            stadium_shape_graphic,
            stadium_shape_text_graphic,
            (name, stadium_shape),
        ) in zip(
            self.stadium_shape_graphics,
            self.stadium_shape_name_graphics,
            stadium_named_shapes,
        ):
            stadium_shape_graphic.update_from_stadium(stadium_shape)

            segment = stadium_shape.segment
            lower_point = (
                segment.start
                if segment.start.y_meters < segment.end.y_meters
                else segment.end
            )
            stadium_shape_text_graphic.setData(
                text=name,
                pos=[
                    lower_point.x_meters,
                    lower_point.y_meters - stadium_shape.radius - 0.1,
                    0,
                ],
            )
