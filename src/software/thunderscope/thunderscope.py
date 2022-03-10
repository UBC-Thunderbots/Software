import os
import signal

import pyqtgraph as pg
from proto.geometry_pb2 import Circle, Polygon
from proto.visualization_pb2 import NamedValue, Obstacles, PathVisualization
from proto.robot_log_msg_pb2 import RobotLog
from proto.world_pb2 import World
from pyqtgraph.dockarea import *
from pyqtgraph.Qt import QtCore, QtGui

from software.networking import threaded_unix_sender
from software.thunderscope.arbitrary_plot.named_value_plotter import NamedValuePlotter
from software.thunderscope.field import (
    obstacle_layer,
    path_layer,
    validation_layer,
    world_layer,
)
from software.thunderscope.field.field import Field
from software.thunderscope.log.g3log_widget import g3logWidget
from software.thunderscope.proto_receiver import ProtoReceiver


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

        self.proto_receiver = ProtoReceiver()

        field_dock = self.setup_field_widget()
        log_dock = self.setup_log_widget()
        performance_dock = self.setup_performance_plot()

        # Configure Docks
        self.dock_area.addDock(field_dock, "left")
        self.dock_area.addDock(log_dock, "bottom", field_dock)
        self.dock_area.addDock(performance_dock, "right", log_dock)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(refresh_interval_ms)  # Refresh at 200hz

    def setup_field_widget(self):
        """TODO: Docstring for setup_field.

        :param function: TODO
        :returns: TODO

        """
        self.field = Field()

        world = world_layer.WorldLayer()
        obstacles = obstacle_layer.ObstacleLayer()
        paths = path_layer.PathLayer()
        validation = validation_layer.ValidationLayer()

        self.field.add_layer("Vision", world)
        self.field.add_layer("Obstacles", obstacles)
        self.field.add_layer("Paths", paths)
        self.field.add_layer("Validation", validation)

        self.proto_receiver.register_observer(World, world.world_buffer)
        self.proto_receiver.register_observer(Obstacles, obstacles.obstacle_buffer)
        self.proto_receiver.register_observer(
            PathVisualization, paths.path_visualization_buffer
        )

        field_dock = Dock("Field", size=(500, 2000))
        field_dock.addWidget(self.field)

        return field_dock

    def setup_log_widget(self):
        """TODO: Docstring for setup_log_widget.

        :param function: TODO
        :returns: TODO

        """
        self.logs = g3logWidget()
        self.proto_receiver.register_observer(RobotLog, self.logs.log_buffer)

        log_dock = Dock("logs", size=(500, 100))
        log_dock.addWidget(self.logs)

        return log_dock

    def setup_performance_plot(self):
        self.named_value_plotter = NamedValuePlotter()
        self.named_value_plotter_dock = Dock("Performance", size=(500, 100))
        self.named_value_plotter_dock.addWidget(self.named_value_plotter.plot)

        self.proto_receiver.register_observer(
            NamedValue, self.named_value_plotter.named_value_buffer
        )

        named_value_plotter_dock = Dock("Performance", size=(500, 100))
        named_value_plotter_dock.addWidget(self.named_value_plotter.plot)

        return named_value_plotter_dock

    def refresh(self):
        self.field.refresh()
        self.logs.refresh()
        self.named_value_plotter.refresh()

    def show(self):
        self.window.show()
        pg.exec()

    def close(self):
        self.window.close()


if __name__ == "__main__":
    thunderscope = Thunderscope()
    thunderscope.show()
