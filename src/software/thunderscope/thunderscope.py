import os
import signal
import time
import shelve
import argparse

import platform

# PyQt5 doesn't play nicely with i3 and Ubuntu 18, PyQt6 is much more stable
# Unfortunately, PyQt6 doesn't install on Ubuntu 18. Thankfully both
# libraries are interchangeable, and  we just need to swap them in this
# one spot, and pyqtgraph will pick up on it and store the library under
# pyqtgraph.Qt. So from PyQt5 import x becomes from pyqtgraph.Qt import x
if "18.04" in platform.version():
    import PyQt5
else:
    import PyQt6

import pyqtgraph
from pyqtgraph.dockarea import *
from pyqtgraph.parametertree import Parameter, ParameterTree
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *

from proto.import_all_protos import *
from proto.message_translation.message_to_dict import message_to_dict

from software.networking import threaded_unix_sender
from software.thunderscope.arbitrary_plot.named_value_plotter import NamedValuePlotter
from software.thunderscope.common.proto_configuration_widget import (
    ProtoConfigurationWidget,
)
from software.thunderscope.field import (
    obstacle_layer,
    path_layer,
    validation_layer,
    world_layer,
)
from software.thunderscope.field.field import Field
from software.thunderscope.log.g3log_widget import g3logWidget
from software.thunderscope.proto_receiver import ProtoReceiver
from software.thunderscope.play.playinfo_widget import playInfoWidget


class Thunderscope(object):

    """ Thunderscope is our main visualizer that can visualize our field,
    obstacles, paths, performance metrics, logs, plots. Thunderscope also
    provides tools to interact with the robots.

    Thunderscope uses pyqtgraph, which is highly configurable during runtime.
    Users can move docks (purple bar) around, double click to pop them out into
    another window, etc.

    The setup_* functions return docks. See configure_default_layout for an
    example. The returned docks can be arranged differently based on the
    use case (robot diagnostics, simulation, robocup, demo, etc..)

    """

    def __init__(self, refresh_interval_ms=5):

        # Setup MainApp and initialize DockArea
        self.app = pyqtgraph.mkQApp("Thunderscope")
        self.app.setStyleSheet(qdarktheme.load_stylesheet())

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
        self.refresh_functions = []

        def __refresh():
            for refresh_func in self.refresh_functions:
                refresh_func()

        # Setup refresh Timer
        self.refresh_timer = QtCore.QTimer()
        self.refresh_timer.setTimerType(QtCore.Qt.TimerType.PreciseTimer)
        self.refresh_timer.timeout.connect(__refresh)
        self.refresh_timer.start(refresh_interval_ms)  # Refresh at 200hz


        def __save_layout():

            filename, _ = QtGui.QFileDialog.getSaveFileName(
                    self.window,
                    "Save layout",
                    "~/dock_layout_{}.tscopelayout".format(int(time.time())),
                    options=QFileDialog.Option.DontUseNativeDialog,
            )

            result = self.dock_area.saveState()

            with shelve.open(filename, 'c') as shelf:
                shelf['dock_state'] = result

        def __load_layout():

            filename, _ = QtGui.QFileDialog.getOpenFileName(
                    self.window,
                    "Open layout", "~/",
                    options=QFileDialog.Option.DontUseNativeDialog,
            )

            with shelve.open(filename, 'r') as shelf:
                self.dock_area.restoreState(shelf['dock_state'])


        self.save_layout = QtGui.QShortcut(QtGui.QKeySequence('Ctrl+S'), self.window)
        self.save_layout.activated.connect(__save_layout)

        self.save_layout = QtGui.QShortcut(QtGui.QKeySequence('Ctrl+O'), self.window)
        self.save_layout.activated.connect(__load_layout)

        self.show_help = QtGui.QShortcut(QtGui.QKeySequence('h'), self.window)
        self.show_help.activated.connect(lambda : QMessageBox.information(self.window,
            'Help', """
                Cntrl+S: Save Layout
                Double Click Purple Bar to pop window out
                Drag Purple Bar to rearrange docks
                Click items in legends to select/deselect
                """))

    def register_refresh_function(self, refresh_func):
        """Register the refresh functions to run at the refresh_interval_ms
        passed into thunderscope.

        :param refresh_func: The function to call at refresh_interval_ms

        """
        self.refresh_functions.append(refresh_func)

    def configure_default_layout(self):
        """Configure the default layout for thunderscope
        """
        # Configure Docks
        field_dock = self.setup_field_widget()
        log_dock = self.setup_log_widget()
        parameter_dock = self.setup_parameter_widget()
        performance_dock = self.setup_performance_plot()
        play_info_dock = self.setup_play_info()

        self.dock_area.addDock(field_dock, "left")
        self.dock_area.addDock(log_dock, "bottom", field_dock)
        self.dock_area.addDock(performance_dock, "right", log_dock)
        self.dock_area.addDock(parameter_dock, "top", log_dock)
        self.dock_area.addDock(play_info_dock, "right", performance_dock)

    def setup_field_widget(self):
        """Setup the field widget with the constituent layers

        :returns: The dock containing the field widget

        """
        self.field = Field()

        # Create layers
        world = world_layer.WorldLayer()
        obstacles = obstacle_layer.ObstacleLayer()
        paths = path_layer.PathLayer()
        validation = validation_layer.ValidationLayer()

        # Add field layers to field
        self.field.add_layer("Vision", world)
        self.field.add_layer("Obstacles", obstacles)
        self.field.add_layer("Paths", paths)
        self.field.add_layer("Validation", validation)

        # Register observers
        self.proto_receiver.register_observer(World, world.world_buffer)
        self.proto_receiver.register_observer(Obstacles, obstacles.obstacle_buffer)
        self.proto_receiver.register_observer(
            PathVisualization, paths.path_visualization_buffer
        )

        # Register refresh functions
        self.register_refresh_function(self.field.refresh)

        # Create and return dock
        field_dock = Dock("Field", size=(500, 3000))
        field_dock.addWidget(self.field)

        return field_dock

    def setup_parameter_widget(self):
        def cak(p, t):
            print(p, t)

        print(ThunderbotsConfig)
        p = ProtoConfigurationWidget(ThunderbotsConfig, cak)

        # Create and return dock
        param_dock = Dock("Parameters", size=(20, 100))
        param_dock.addWidget(p)

        return param_dock

    def setup_log_widget(self):
        """Setup the wiget that receives logs from full system

        :returns: The dock containing the log widget

        """
        # Create layout
        layout = QVBoxLayout()
        widget = QWidget()

        # Create widget
        self.logs = g3logWidget()

        # Register observer
        self.proto_receiver.register_observer(RobotLog, self.logs.log_buffer)

        # Register refresh function
        self.register_refresh_function(self.logs.refresh)

        # Setup Checkbox Widget
        layout.addWidget(self.logs)
        layout.addWidget(self.logs.checkbox_widget)
        widget.setLayout(layout)

        # Create and return dock
        log_dock = Dock("Logs", size=(50, 100))
        log_dock.addWidget(widget)

        return log_dock

    def setup_performance_plot(self):
        """Setup the performance plot

        :returns: The performance plot setup in a dock

        """
        # Create widget
        self.named_value_plotter = NamedValuePlotter()

        # Register observer
        self.proto_receiver.register_observer(
            NamedValue, self.named_value_plotter.named_value_buffer
        )

        # Register refresh funcntion
        self.register_refresh_function(self.named_value_plotter.refresh)

        # Create and return dock
        named_value_plotter_dock = Dock("Performance", size=(500, 100))
        named_value_plotter_dock.addWidget(self.named_value_plotter.plot)
        return named_value_plotter_dock

    def setup_play_info(self):
        """Setup the play info widget

        :returns: The play info widget setup in a dock

        """

        play_info = playInfoWidget()
        play_info_dock = Dock("playInfo", size=(500, 100))
        play_info_dock.addWidget(play_info)
        self.proto_receiver.register_observer(PlayInfo, play_info.log_buffer)
        self.register_refresh_function(play_info.refresh)
        return play_info_dock

    def show(self):
        self.window.show()
        pyqtgraph.exec()

    def close(self):
        QtCore.QTimer.singleShot(0, self.window.close)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Thunderscope")
    parser.add_argument(
        "--robot_diagnostics",
        action="store_true",
        help="Run thunderscope in the robot diagnostics configuration",
    )
    parser.add_argument(
        "--run_simulator", action="store_true", help="Run the standalone simulator"
    )

    args = parser.parse_args()

    if args.robot_diagnostics:
        thunderscope = Thunderscope()

        log_dock = thunderscope.setup_log_widget()
        thunderscope.dock_area.addDock(log_dock)

        thunderscope.show()

    elif args.run_simulator:
        print(
            "TODO #2050, this isn't implemented, just run the current standalone simulator"
        )

    else:
        thunderscope = Thunderscope()
        thunderscope.configure_default_layout()
        thunderscope.show()
