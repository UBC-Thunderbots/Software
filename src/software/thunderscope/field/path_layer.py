import pyqtgraph as pg
from proto.visualization_pb2 import PathVisualization
from pyqtgraph.Qt import QtCore, QtGui

import software.thunderscope.constants as constants
from software.thunderscope.colors import Colors
from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.thunderscope.field.field_layer import FieldLayer
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class PathLayer(FieldLayer):
    def __init__(self, buffer_size=5):
        """Visualizes paths from the navigator

        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        FieldLayer.__init__(self)
        self.path_visualization_buffer = ThreadSafeBuffer(
            buffer_size, PathVisualization
        )

    def paint(self, painter, option, widget):
        """Paint this layer

        :param painter: The painter object to draw with
        :param option: Style information (unused)
        :param widget: The widget that we are painting on

        """
        paths = self.path_visualization_buffer.get(block=False)
        painter.setPen(
            pg.mkPen(Colors.NAVIGATOR_PATH_COLOR, width=constants.LINE_WIDTH)
        )

        for path in paths.path:
            polygon_points = [
                QtCore.QPoint(
                    int(constants.MM_PER_M * point.x_meters),
                    int(constants.MM_PER_M * point.y_meters),
                )
                for point in path.point
            ]
            poly = QtGui.QPolygon(polygon_points)
            painter.drawPolyline(poly)
