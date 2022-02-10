import pyqtgraph as pg
from software.thunderscope.field.field_layer import FieldLayer
import software.thunderscope.constants as constants
from pyqtgraph.Qt import QtCore, QtGui
from proto.visualization_pb2 import PathVisualization
from software.networking.threaded_unix_listener import ThreadedUnixListener


class PathLayer(FieldLayer):
    def __init__(self):
        FieldLayer.__init__(self)
        self.path_receiver = ThreadedUnixListener(
            constants.UNIX_SOCKET_BASE_PATH + PathVisualization.DESCRIPTOR.full_name,
            PathVisualization
        )
        self.cached_paths = PathVisualization()

    def paint(self, painter, option, widget):
        """Paint this layer

        :param painter: The painter object to draw with
        :param option: Style information (unused)
        :param widget: The widget that we are painting on

        """

        paths = self.path_receiver.maybe_pop()

        if not paths:
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
