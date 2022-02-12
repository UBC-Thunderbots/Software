import os
import signal

from software.thunderscope.field.field import Field
from software.thunderscope.log.g3log_widget import g3logWidget
from field import obstacle_layer, path_layer, world_layer

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.dockarea import *


if __name__ == "__main__":

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

    # Setup 'Chicker' widget
    chicker_dock = Dock("chicker", size=(100,100))
    # chicker_dock.addWidget(create a new widget)

    # Configure Docks
    dock_area.addDock(field_dock, "left")
    dock_area.addDock(log_dock, "bottom", field_dock)

    def update():
        field.refresh()
        logs.refresh()

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(5)  # Refresh at 200hz

    window.show()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    pg.exec()
