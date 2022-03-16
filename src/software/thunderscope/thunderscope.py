import os
import signal

import software.thunderscope.constants as constants
import queue
import software.thunderscope.colors as colors
from proto.geometry_pb2 import Polygon, Circle
from proto.visualization_pb2 import Obstacles
from proto.visualization_pb2 import PathVisualization
from software.thunderscope.field.field import Field
from proto.world_pb2 import World
from proto.team_pb2 import Robot, Team
from proto.ball_pb2 import Ball
from threading import Thread
from software.thunderscope.log.g3log_widget import g3logWidget
from software.thunderscope.log.g3log_checkboxes import g3logCheckboxes
from software.thunderscope.arbitrary_plot.named_value_plotter import NamedValuePlotter
from proto.visualization_pb2 import NamedValue
from proto.robot_log_msg_pb2 import RobotLog
from software.networking.threaded_unix_listener import ThreadedUnixListener
from field import obstacle_layer, path_layer, world_layer
from proto_receiver import ProtoReceiver
import software.thunderscope.constants as constants
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.dockarea import *


if __name__ == "__main__":

    # Setup unix socket directory
    try:
        os.mkdir("/tmp/tbots")
    except Exception as e:
        print(e)
        pass

    proto_receiver = ProtoReceiver()

    # Setup MainApp and initialize DockArea
    app = pg.mkQApp("Thunderscope")
    app.setStyleSheet("QMainWindow{background-color: black;border: 1px solid black;}")
    window = QtGui.QMainWindow()
    dock_area = DockArea()
    window.setCentralWidget(dock_area)
    window.setWindowTitle("Thunderscope")

    # Setup Field + Layers
    field = Field()
    world = world_layer.WorldLayer()
    obstacles = obstacle_layer.ObstacleLayer()
    paths = path_layer.PathLayer()

    proto_receiver.register_observer(World, world.world_buffer)
    proto_receiver.register_observer(Obstacles, obstacles.obstacle_buffer)
    proto_receiver.register_observer(PathVisualization, paths.path_visualization_buffer)

    field.add_layer("Vision", world)
    field.add_layer("Obstacles", obstacles)
    field.add_layer("Path", paths)

    field_dock = Dock("Field", size=(500, 2000))
    field_dock.addWidget(field)

    # Setup Console Widget
    logs = g3logWidget()
    proto_receiver.register_observer(RobotLog, logs.log_buffer)

    log_dock = Dock("logs", size=(500, 100))
    log_dock.addWidget(logs)

    # Setup Checkbox Widget
    check_boxes = logs.checkboxWidget

    check_boxes_dock = Dock("Logs filter", size=(100, 100))
    check_boxes_dock.addWidget(check_boxes)
    # Setup Arbitrary Plot Widget
    named_value_plotter = NamedValuePlotter()
    named_value_plotter_dock = Dock("Performance", size=(500, 100))
    named_value_plotter_dock.addWidget(named_value_plotter.plot)
    proto_receiver.register_observer(NamedValue, named_value_plotter.named_value_buffer)

    # Configure Docks
    dock_area.addDock(field_dock, "left")
    dock_area.addDock(log_dock, "bottom", field_dock)
    dock_area.addDock(check_boxes_dock, "bottom", log_dock)
    dock_area.addDock(named_value_plotter_dock, "right", log_dock)

    def update():
        field.refresh()
        logs.refresh()
        named_value_plotter.refresh()

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(5)  # Refresh at 200hz

    window.show()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    pg.exec()
