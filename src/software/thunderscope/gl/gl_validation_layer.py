from typing import List

from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl import *

import time
import numpy as np

from proto.import_all_protos import *

from software.py_constants import *
from software.thunderscope.constants import Colors
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.gl.gl_layer import GLLayer
from software.thunderscope.gl.graphics.gl_circle import GLCircle


class GLValidationLayer(GLLayer):
    """GLLayer that visualizes validation"""

    PASSED_VALIDATION_PERSISTANCE_TIMEOUT_S = 1.0

    def __init__(
        self,
        name: str,
        buffer_size: int = 10,
        test_name_pos_x: float = -4.5,
        test_name_pos_y: float = 3.2,
    ):
        """Initialize the GLValidationLayer

        :param name: The displayed name of the layer
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary
        :param test_name_pos_x: The x position of the test name
        :param test_name_pos_y: The y position of the test name

        """
        GLLayer.__init__(self, name)

        # Validation protobufs are generated by simulated tests
        self.validation_set_buffer = ThreadSafeBuffer(buffer_size, ValidationProtoSet)
        self.cached_always_validation_set = ValidationProtoSet()
        self.cached_eventually_validation_set = ValidationProtoSet()

        self.passed_validation_timeout_pairs = []

        self.graphics_list.register_graphics_group(
            "test_name",
            lambda: GLTextItem(
                pos=(test_name_pos_x, test_name_pos_y, 0),
                font=QtGui.QFont("Roboto", 8),
                color=Colors.PRIMARY_TEXT_COLOR,
            ),
        )
        self.graphics_list.register_graphics_group(
            "validation_polygons", GLLinePlotItem
        )
        self.graphics_list.register_graphics_group("validation_circles", GLCircle)

    def _update_graphics(self):
        """Fetch and update graphics for the layer"""

        # Consume the validation set buffer
        for _ in range(self.validation_set_buffer.queue.qsize()):
            self.validation_set = self.validation_set_buffer.get()

            if self.validation_set.validation_type == ValidationType.ALWAYS:
                self.cached_always_validation_set = self.validation_set
            else:
                self.cached_eventually_validation_set = self.validation_set

        # Cache passed validations
        for validation in self.cached_eventually_validation_set.validations:
            if validation.status == ValidationStatus.PASSING:
                self.passed_validation_timeout_pairs.append(
                    (
                        validation,
                        time.time()
                        + GLValidationLayer.PASSED_VALIDATION_PERSISTANCE_TIMEOUT_S,
                    )
                )

        # Remove timed out cached validations
        for validation, stop_drawing_time in list(self.passed_validation_timeout_pairs):
            if time.time() >= stop_drawing_time:
                self.passed_validation_timeout_pairs.remove(
                    (validation, stop_drawing_time)
                )

        self.__update_validation_graphics(
            list(self.cached_always_validation_set.validations)
            + list(self.cached_eventually_validation_set.validations)
            + list(self.passed_validation_timeout_pairs)
        )

        test_name_graphic = self.graphics_list.get_graphics("test_name", 1)[0]
        test_name_graphic.setData(text=self.cached_eventually_validation_set.test_name)

    def __update_validation_graphics(self, validations: List[ValidationProto]):
        """Update the GLGraphicsItems that display the validations
        
        :param validations: The list of validation protos

        """
        for validation in validations:

            validation_color = (
                Colors.VALIDATION_PASSED_COLOR
                if validation.status == ValidationStatus.PASSING
                else Colors.VALIDATION_FAILED_COLOR
            )

            for polygon_graphic, polygon in zip(
                self.graphics_list.get_graphics(
                    "validation_polygons", len(validation.geometry.polygons)
                ),
                validation.geometry.polygons,
            ):
                # In order to close the polygon, we need to include the first point at the end of
                # the list of points in the polygon
                polygon_points = list(polygon.points) + polygon.points[:1]

                polygon_graphic.setData(
                    pos=np.array(
                        [
                            [point.x_meters, point.y_meters, 0]
                            for point in polygon_points
                        ]
                    ),
                    color=validation_color,
                )

            for segment_graphic, segment in zip(
                self.graphics_list.get_graphics(
                    "validation_polygons", len(validation.geometry.segments)
                ),
                validation.geometry.segments,
            ):
                segment_graphic.setData(
                    pos=np.array(
                        [
                            [segment.start.x_meters, segment.start.y_meters],
                            [segment.end.x_meters, segment.end.y_meters],
                        ]
                    ),
                    color=validation_color,
                )

            for circle_graphic, circle in zip(
                self.graphics_list.get_graphics(
                    "validation_circles", len(validation.geometry.circles)
                ),
                validation.geometry.circles,
            ):
                circle_graphic.set_radius(circle.radius)
                circle_graphic.set_position(
                    circle.origin.x_meters, circle.origin.y_meters
                )
                circle_graphic.set_color(validation_color)
