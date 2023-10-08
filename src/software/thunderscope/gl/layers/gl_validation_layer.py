from typing import List

from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl import *

import time
import numpy as np

from proto.import_all_protos import *

from software.py_constants import *
from software.thunderscope.constants import Colors
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.gl.graphics.gl_circle import GLCircle

from software.thunderscope.gl.helpers.observable_list import ObservableList


class GLValidationLayer(GLLayer):
    """GLLayer that visualizes validation"""

    PASSED_VALIDATION_PERSISTANCE_TIMEOUT_S = 1.0

    def __init__(
        self,
        name: str,
        buffer_size: int = 10,
        test_name_pos_x: float = -4.5,
        test_name_pos_y: float = 3.6,
    ):
        """Initialize the GLValidationLayer

        :param name: The displayed name of the layer
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary
        :param test_name_pos_x: The x position of the test name
        :param test_name_pos_y: The y position of the test name

        """
        super().__init__(name)

        # Validation protobufs are generated by simulated tests
        self.validation_set_buffer = ThreadSafeBuffer(buffer_size, ValidationProtoSet)
        self.cached_always_validation_set = ValidationProtoSet()
        self.cached_eventually_validation_set = ValidationProtoSet()

        self.passed_validation_timeout_pairs = []

        self.test_name_pos_x = test_name_pos_x
        self.test_name_pos_y = test_name_pos_y

        # GLTextItem must be initialized later, outside of this constructor
        # Otherwise we run into some strange bugs: 'NoneType' object has no attribute 'width'
        self.test_name_graphic: GLTextItem = None

        self.polygon_graphics = ObservableList(self._graphics_changed)
        self.segment_graphics = ObservableList(self._graphics_changed)
        self.circle_graphics = ObservableList(self._graphics_changed)

    def refresh_graphics(self):
        """Update graphics in this layer"""

        if not self.test_name_graphic:
            self.test_name_graphic = GLTextItem(
                parentItem=self,
                pos=(self.test_name_pos_x, self.test_name_pos_y, 0),
                font=QtGui.QFont("Roboto", 8),
                color=Colors.PRIMARY_TEXT_COLOR,
            )

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

        self.test_name_graphic.setData(
            text=self.cached_eventually_validation_set.test_name
        )

    def __update_validation_graphics(self, validations: List[ValidationProto]):
        """Update the GLGraphicsItems that display the validations
        
        :param validations: The list of validation protos

        """
        polygons = [
            (polygon, validation.status)
            for validation in validations
            for polygon in validation.geometry.polygons
        ]
        segments = [
            (segment, validation.status)
            for validation in validations
            for segment in validation.geometry.segments
        ]
        circles = [
            (circle, validation.status)
            for validation in validations
            for circle in validation.geometry.circles
        ]

        # Ensure we have the same number of graphics as validations
        self._bring_list_to_length(
            self.polygon_graphics,
            len(polygons),
            lambda: GLLinePlotItem(width=3.0),
        )
        self._bring_list_to_length(
            self.segment_graphics,
            len(segments),
            lambda: GLLinePlotItem(width=3.0),
        )
        self._bring_list_to_length(
            self.circle_graphics,
            len(circles),
            lambda: GLCircle(),
        )

        for polygon_graphic, (polygon, validation_status) in zip(
            self.polygon_graphics, polygons
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
                color=self.__get_validation_color(validation_status),
            )

        for segment_graphic, (segment, validation_status) in zip(
            self.segment_graphics, segments
        ):
            segment_graphic.setData(
                pos=np.array(
                    [
                        [segment.start.x_meters, segment.start.y_meters],
                        [segment.end.x_meters, segment.end.y_meters],
                    ]
                ),
                color=self.__get_validation_color(validation_status),
            )

        for circle_graphic, (circle, validation_status) in zip(
            self.circle_graphics, circles
        ):
            circle_graphic.set_radius(circle.radius)
            circle_graphic.set_position(
                circle.origin.x_meters, circle.origin.y_meters
            )
            circle_graphic.set_color(self.__get_validation_color(validation_status))

    def __get_validation_color(self, validation_status: ValidationStatus):
        """Get the color representing the given validation status
        
        :param validation_status: the validation status

        """ 
        return (
            Colors.VALIDATION_PASSED_COLOR
            if validation_status == ValidationStatus.PASSING
            else Colors.VALIDATION_FAILED_COLOR
        )