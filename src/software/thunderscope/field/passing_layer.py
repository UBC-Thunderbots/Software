import pyqtgraph as pg
import time
from software.thunderscope.field.field_layer import FieldLayer
import software.thunderscope.constants as constants
from pyqtgraph.Qt import QtCore, QtGui
from proto.visualization_pb2 import PassVisualization
from software.networking.threaded_unix_listener import ThreadedUnixListener


class PassingLayer(FieldLayer):

    PASS_VISUALIZATION_TIMEOUT_S = 0.5
    NUM_TOP_PASSES_TO_RENDER = 3

    def __init__(self):
        FieldLayer.__init__(self)
        self.pass_visualization_receiver = ThreadedUnixListener(
            constants.UNIX_SOCKET_BASE_PATH + PassVisualization.DESCRIPTOR.full_name,
            PassVisualization,
        )

        self.cached_pass_vis = PassVisualization()
        self.timeout = time.time() + PassingLayer.PASS_VISUALIZATION_TIMEOUT_S

    def paint(self, painter, option, widget):
        """Paint this layer

        :param painter: The painter object to draw with
        :param option: Style information (unused)
        :param widget: The widget that we are painting on

        """

        pass_vis = self.pass_visualization_receiver.maybe_pop()

        if not pass_vis:
            pass_vis = self.cached_pass_vis

            # If we haven't received pass visualizations for a bit, clear the layer
            if time.time() > self.timeout:
                return
        else:
            # we received new pass data, so lets update our timeout
            self.timeout = time.time() + PassingLayer.PASS_VISUALIZATION_TIMEOUT_S
            self.cached_pass_vis = pass_vis

        sorted_pass_with_rating = sorted(
            pass_vis.best_passes, key=lambda x: x.rating, reverse=True
        )

        for pass_num, pass_with_rating in enumerate(
            sorted_pass_with_rating[0 : PassingLayer.NUM_TOP_PASSES_TO_RENDER]
        ):

            # Lets color the best pass green and rest dark pink
            if pass_num == 0:
                painter.setPen(pg.mkPen("green"))
            else:
                painter.setPen(pg.mkPen("pink"))

            painter.drawEllipse(
                self.createCircle(
                    pass_with_rating.pass_.receiver.x_meters * constants.MM_PER_M,
                    pass_with_rating.pass_.receiver.y_meters * constants.MM_PER_M,
                    constants.ROBOT_MAX_RADIUS * 2.5,
                )
            )
