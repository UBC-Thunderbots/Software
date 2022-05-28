import pyqtgraph as pg
import time
import queue
from software.thunderscope.field.field_layer import FieldLayer
import software.thunderscope.constants as constants
from software.py_constants import *
from pyqtgraph.Qt import QtCore, QtGui
from proto.import_all_protos import *
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class PassingLayer(FieldLayer):

    # The pass generator is created and destroyed as we move in and out of offensive  plays.
    # If we no longer receive new passes, we need to stop drawing the old one.
    PASS_VISUALIZATION_TIMEOUT_S = 0.5

    # We receive a list of passes, ranked by their score. We want to render the top
    # NUM_TOP_PASSES_TO_RENDER passes.
    NUM_TOP_PASSES_TO_RENDER = 3

    def __init__(self, buffer_size=5):
        """Initializes the passing layer

        :param buffer_size: The size of the buffer to use for the pass visualization

        """

        FieldLayer.__init__(self)
        self.pass_visualization_buffer = ThreadSafeBuffer(
            buffer_size, PassVisualization
        )
        self.cached_pass_vis = PassVisualization()
        self.timeout = time.time() + PassingLayer.PASS_VISUALIZATION_TIMEOUT_S

        # Test item to show scores
        self.pass_rating_text_items = [
            pg.TextItem() for _ in range(PassingLayer.NUM_TOP_PASSES_TO_RENDER)
        ]

        for text_item in self.pass_rating_text_items:
            text_item.setParentItem(self)

    def paint(self, painter, option, widget):
        """Paint this layer

        :param painter: The painter object to draw with
        :param option: Style information (unused)
        :param widget: The widget that we are painting on

        """

        try:
            pass_vis = self.pass_visualization_buffer.queue.get_nowait()
        except queue.Empty as empty:
            pass_vis = None

        if not pass_vis:
            pass_vis = self.cached_pass_vis

            for text_item in self.pass_rating_text_items:
                text_item.setText("")

            # If we haven't received pass visualizations for a bit, clear the layer
            if time.time() > self.timeout:
                return
        else:
            # We received new pass data, so lets update our timeout
            self.timeout = time.time() + PassingLayer.PASS_VISUALIZATION_TIMEOUT_S
            self.cached_pass_vis = pass_vis

        sorted_pass_with_rating = sorted(
            pass_vis.best_passes, key=lambda x: x.rating, reverse=True
        )

        for pass_num, pass_with_rating in enumerate(
            sorted_pass_with_rating[0 : PassingLayer.NUM_TOP_PASSES_TO_RENDER]
        ):

            # Lets color the best pass green and rest white
            painter.setPen(
                pg.mkPen(
                    "green" if pass_num == 0 else "white", width=constants.LINE_WIDTH
                )
            )

            painter.drawEllipse(
                self.createCircle(
                    pass_with_rating.pass_.receiver_point.x_meters
                    * MILLIMETERS_PER_METER,
                    pass_with_rating.pass_.receiver_point.y_meters
                    * MILLIMETERS_PER_METER,
                    ROBOT_MAX_RADIUS_MILLIMETERS * 4,
                )
            )

            # Draw the pass rating
            text_item = self.pass_rating_text_items[pass_num]
            text_item.setText(
                str("Pass {} {:.2f}".format(pass_num, pass_with_rating.rating))
            )
            text_item.setPos(
                pass_with_rating.pass_.receiver_point.x_meters * MILLIMETERS_PER_METER
                - 4 * ROBOT_MAX_RADIUS_MILLIMETERS,
                pass_with_rating.pass_.receiver_point.y_meters * MILLIMETERS_PER_METER
                - 4 * ROBOT_MAX_RADIUS_MILLIMETERS,
            )
