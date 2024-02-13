from pyqtgraph.opengl import *

from proto.visualization_pb2 import DebugShapeList

from software.thunderscope.constants import Colors, DepthValues
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.gl.graphics.gl_circle import GLCircle
from software.thunderscope.gl.graphics.gl_polygon import GLPolygon
from software.thunderscope.gl.graphics.gl_stadium import GLStadium

import math

from software.thunderscope.gl.helpers.observable_list import ObservableList


class GLDebugShapesLayer(GLLayer):
    """GLLayer that visualizes shapes"""

    def __init__(self, name: str, buffer_size: int = 1) -> None:
        """Initialize the GLDebugShapeLayer

        :param name: The displayed name of the layer
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        super().__init__(name)
        self.setDepthValue(DepthValues.BACKGROUND_DEPTH)

        self.debug_shape_list_buffer = ThreadSafeBuffer(buffer_size, DebugShapeList)

        self.poly_shape_graphics = ObservableList(self._graphics_changed)
        self.circle_shape_graphics = ObservableList(self._graphics_changed)
        self.stadium_shape_graphics = ObservableList(self._graphics_changed)

        self.drawn_

    def refresh_graphics(self) -> None:
        """Update graphics in this layer"""
        named_shapes = self.debug_shape_list_buffer.get(block=False).shapes
        # TODO (NIMA): Keep track of last plotted value time. Could distinguish between the protos using their names. Per name it can timeout
        poly_shapes = []
        circle_shapes = []
        stadium_shapes = []
        for named_shape in named_shapes:
            shape = named_shape.shape
            if shape.HasField("polygon"):
                poly_shapes.append(shape.polygon)
            elif shape.HasField("stadium"):
                stadium_shapes.append(shape.stadium)
            else:
                circle_shapes.append(shape.circle)

        # Ensure we have the same number of graphics as shapes
        self.poly_shape_graphics.resize(
            len(poly_shapes),
            lambda: GLPolygon(outline_color=Colors.DEBUG_SHAPES_COLOR), # TODO (NIMA): Change color
        )
        self.circle_shape_graphics.resize(
            len(circle_shapes),
            lambda: GLCircle(outline_color=Colors.DEBUG_SHAPES_COLOR),
        )
        self.stadium_shape_graphics.resize(
            len(stadium_shapes),
            lambda: GLStadium(outline_color=Colors.DEBUG_SHAPES_COLOR),
        )

        for poly_shape_graphic, poly_shape in zip(
            self.poly_shape_graphics, poly_shapes
        ):
            # In order to close the polygon, we need to include the first point at the end of
            # the list of points in the polygon
            polygon_points = list(poly_shape.points) + poly_shape.points[:1]

            poly_shape_graphic.set_points(
                [[point.x_meters, point.y_meters] for point in polygon_points]
            )

        for circle_shape_graphic, circle_shape in zip(
            self.circle_shape_graphics, circle_shapes
        ):
            circle_shape_graphic.set_radius(circle_shape.radius)
            circle_shape_graphic.set_position(
                circle_shape.origin.x_meters, circle_shape.origin.y_meters,
            )

        for stadium_shape_graphic, stadium_shape in zip(
            self.stadium_shape_graphics, stadium_shapes
        ):
            # set basic parameters
            x_start_to_end = (
                stadium_shape.segment.end.x_meters
                - stadium_shape.segment.start.x_meters
            )
            y_start_to_end = (
                stadium_shape.segment.end.y_meters
                - stadium_shape.segment.start.y_meters
            )
            length = math.sqrt(
                math.pow(x_start_to_end, 2) + math.pow(y_start_to_end, 2)
            )
            stadium_shape_graphic.set_parameters(stadium_shape.radius, length)
            # set stadium position to average of its two points
            stadium_shape_graphic.set_position(
                (
                    stadium_shape.segment.end.x_meters
                    + stadium_shape.segment.start.x_meters
                )
                / 2,
                (
                    stadium_shape.segment.end.y_meters
                    + stadium_shape.segment.start.y_meters
                )
                / 2,
            )
            # set stadium orientation to angle between positive x and vector from start to end
            stadium_shape_graphic.set_orientation(
                math.atan2(y_start_to_end, x_start_to_end) * 180 / math.pi
            )
