import os
import signal

from software.thunderscope.field.field import Field
from software.thunderscope.log.g3log_widget import g3logWidget
from software.thunderscope.arbitrary_plot.arb_plot import NamedValuePlotter
from field import obstacle_layer, path_layer, world_layer

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.dockarea import *


if __name__ == "__main__":

    class ProtoReceiver():
        def __init__(self):
            self.proto_map = dict()
            self.proto_receiver = ThreadedUnixListener(
                constants.UNIX_SOCKET_BASE_PATH + "protobuf", convert_from_any=True, max_buffer_size=3
            )
            self.thread = Thread(target=self.start)
            self.thread.start()

        def start(self):
            while True: 
                proto = self.proto_receiver.buffer.get()
                print(proto)

        def registerObserver(self, proto_type, buffer):
            if proto_type in self.proto_map:
                self.proto_map[proto_type].append(buffer)
            else:
                self.proto_map[proto_type] = [buffer]
        
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
    field.add_layer("Vision", world_layer.WorldLayer())
    field.add_layer("Obstacles", obstacle_layer.ObstacleLayer())
    field.add_layer("Path", path_layer.PathLayer())

    field_dock = Dock("Field", size=(500, 2000))
    field_dock.addWidget(field)

    # Setup Console Widget
    logs = g3logWidget()

    log_dock = Dock("logs", size=(500, 100))
    log_dock.addWidget(logs)

    #Setup Arbitrary Plot Widget
    arb_plot = NamedValuePlotter()
    arb_plot_dock = Dock("Performance", size = (500,100))
    arb_plot_dock.addWidget(arb_plot.plot)

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
