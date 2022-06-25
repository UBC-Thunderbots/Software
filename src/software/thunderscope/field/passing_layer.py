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

        pen = pg.mkPen(
            QtGui.QColor(255, 0, 0, 80), width=2, style=QtCore.Qt.PenStyle.DashLine
        )
        painter.setPen(pen)

        for pass_with_rating in sorted_pass_with_rating[0:1]:
            polygon_points = [
                QtCore.QPoint(
                    int(
                        pass_with_rating.pass_.passer_point.x_meters
                        * MILLIMETERS_PER_METER
                    ),
                    int(
                        pass_with_rating.pass_.passer_point.y_meters
                        * MILLIMETERS_PER_METER
                    ),
                ),
                QtCore.QPoint(
                    int(
                        pass_with_rating.pass_.receiver_point.x_meters
                        * MILLIMETERS_PER_METER
                    ),
                    int(
                        pass_with_rating.pass_.receiver_point.y_meters
                        * MILLIMETERS_PER_METER
                    ),
                ),
            ]
            poly = QtGui.QPolygon(polygon_points)
            painter.drawPolyline(poly)
