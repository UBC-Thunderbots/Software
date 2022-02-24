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
from software.thunderscope.arbitrary_plot.arb_plot import NamedValuePlotter
from proto.visualization_pb2 import NamedValue
from proto.robot_log_msg_pb2 import RobotLog
from software.networking.threaded_unix_listener import ThreadedUnixListener
from field import obstacle_layer, path_layer, world_layer
import software.thunderscope.constants as constants

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.dockarea import *


if __name__ == "__main__":

    class ProtoReceiver:
        def __init__(self):
            self.proto_map = dict()
            self.proto_receiver = ThreadedUnixListener(
                constants.UNIX_SOCKET_BASE_PATH + "protobuf",
                convert_from_any=True,
                max_buffer_size=3,
            )
            self.thread = Thread(target=self.start)
            self.thread.start()

        def start(self):
            while True:
                proto = self.proto_receiver.buffer.get()
                if proto.DESCRIPTOR.full_name in self.proto_map:
                    for buffer in self.proto_map[proto.DESCRIPTOR.full_name]:
                        try:
                            buffer.put_nowait(proto)
                        except queue.Full:
                            pass

        def registerObserver(self, proto_type, buffer):
            if proto_type in self.proto_map:
                self.proto_map[proto_type.DESCRIPTOR.full_name].append(buffer)
            else:
                self.proto_map[proto_type.DESCRIPTOR.full_name] = [buffer]

    proto_receiver = ProtoReceiver()
    # Setup unix socket directory
    try:
        os.mkdir("/tmp/tbots")
    except:
        pass

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

    proto_receiver.registerObserver(World, world.world_buffer)
    proto_receiver.registerObserver(Obstacles, obstacles.obstacle_buffer)
    proto_receiver.registerObserver(PathVisualization, paths.path_visualization_buffer)

    field.add_layer("Vision", world)
    field.add_layer("Obstacles", obstacles)
    field.add_layer("Path", paths)

    field_dock = Dock("Field", size=(500, 2000))
    field_dock.addWidget(field)

    # Setup Console Widget
    logs = g3logWidget()
    proto_receiver.registerObserver(RobotLog, logs.buffer)

    log_dock = Dock("logs", size=(500, 100))
    log_dock.addWidget(logs)

    # Setup Arbitrary Plot Widget
    arb_plot = NamedValuePlotter()
    arb_plot_dock = Dock("Performance", size=(500, 100))
    arb_plot_dock.addWidget(arb_plot.plot)
    proto_receiver.registerObserver(NamedValue, arb_plot.named_value_buffer)

    # Configure Docks
    dock_area.addDock(field_dock, "left")
    dock_area.addDock(log_dock, "bottom", field_dock)
    dock_area.addDock(arb_plot_dock, "right", log_dock)

    def update():
        field.refresh()
        logs.refresh()
        arb_plot.refresh()

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(3)  # Refresh at 200hz

    window.show()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    pg.exec()
