import pyqtgraph as pg
import queue
from software.thunderscope.field.field_layer import FieldLayer
import software.thunderscope.constants as constants
from pyqtgraph.Qt import QtCore, QtGui
from proto.visualization_pb2 import PathVisualization


class PathLayer(FieldLayer):
    def __init__(self, buffer_size=10):
        FieldLayer.__init__(self)
        self.cached_paths = PathVisualization()
        self.path_visualization_buffer = queue.Queue(buffer_size)

    def paint(self, painter, option, widget):
        """Paint this layer

        :param painter: The painter object to draw with
        :param option: Style information (unused)
        :param widget: The widget that we are painting on

        """
        try:
            paths = self.path_visualization_buffer.get_nowait()
        except queue.Empty as empty:
            paths = self.cached_paths

        self.cached_paths = paths
        painter.setPen(pg.mkPen("w"))

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
