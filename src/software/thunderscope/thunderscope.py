import os
import signal

from software.thunderscope.field.field import Field
from software.thunderscope.log.g3log_widget import g3logWidget
from software.thunderscope.field import obstacle_layer, path_layer, world_layer

from shared.parameter.dynamic_parameters_pb2 import ThunderbotsConfig
from software.networking import threaded_unix_sender

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.dockarea import *

from shared.parameter.dynamic_parameters_pb2 import ThunderbotsConfig


class Thunderscope(object):

    """ Thunderscope """

    def __init__(self, refresh_interval_ms=5):

        # Setup MainApp and initialize DockArea
        self.app = pg.mkQApp("Thunderscope")
        self.app.setStyleSheet(
            "QMainWindow{background-color: black;border: 1px solid black;}"
        )
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        self.dock_area = DockArea()

        self.window = QtGui.QMainWindow()
        self.window.setCentralWidget(self.dock_area)
        self.window.setWindowTitle("Thunderscope")

        # Setup unix socket directory
        try:
            os.mkdir("/tmp/tbots")
        except:
            pass

        field_dock = self.setup_field_widget()
        log_dock = self.setup_log_widget()

        # Configure Docks
        self.dock_area.addDock(field_dock, "left")
        self.dock_area.addDock(log_dock, "bottom", field_dock)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(refresh_interval_ms)  # Refresh at 200hz

    def schedule_something(self, interval, callback):
        self.new_timer = QtCore.QTimer()
        self.new_timer.timeout.connect(callback)
        self.new_timer.start(interval)

    def setup_field_widget(self):
        """TODO: Docstring for setup_field.

        :param function: TODO
        :returns: TODO

        """
        self.field = Field()
        # TODO janky
        self.world_layer = world_layer.WorldLayer()
        self.field.add_layer("Vision", self.world_layer)
        self.field.add_layer("Obstacles", obstacle_layer.ObstacleLayer())
        self.field.add_layer("Path", path_layer.PathLayer())

        field_dock = Dock("Field", size=(500, 2000))
        field_dock.addWidget(self.field)

        return field_dock

    def setup_log_widget(self):
        """TODO: Docstring for setup_log_widget.

        :param function: TODO
        :returns: TODO

        """
        self.logs = g3logWidget()

        log_dock = Dock("logs", size=(500, 100))
        log_dock.addWidget(self.logs)

        return log_dock

    def refresh(self):
        self.field.refresh()
        self.logs.refresh()

    def show(self):
        self.window.show()
        pg.exec()

    def close(self):
        self.window.close()


if __name__ == "__main__":
    thunderscope = Thunderscope()
    thunderscope.show()
