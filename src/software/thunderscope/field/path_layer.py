import pyqtgraph as pg
from software.thunderscope.field.field_layer import FieldLayer
import software.thunderscope.field.constants as constants
from pyqtgraph.Qt import QtCore, QtGui
from proto.visualization_pb2 import PathVisualization
from software.networking.threaded_unix_listener import ThreadedUnixListener


class PathLayer(FieldLayer):
    def __init__(self):
        FieldLayer.__init__(self)
        self.path_receiver = ThreadedUnixListener(
            "/tmp/tbots/TbotsProto.PathVisualization", PathVisualization,
        )
        self.cached_paths = None

    def paint(self, painter, option, widget):

        paths = self.path_receiver.maybe_pop()

        if not paths:
            paths = self.cached_paths

        self.cached_paths = paths
        painter.setPen(pg.mkPen("w"))

        for path in paths.path:
            polygon_points = [
                QtCore.QPoint(
                    int(constants.MM_TO_M * point.x_meters),
                    int(constants.MM_TO_M * point.y_meters),
                )
                for point in path.point
            ]
            poly = QtGui.QPolygon(polygon_points)
            painter.drawPolyline(poly)
