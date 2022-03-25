import os
import time
import signal
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
from pyqtgraph.Qt.QtWidgets import QVBoxLayout, QWidget

from proto.import_all_protos import *
from proto.message_translation.message_to_dict import message_to_dict

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
        self.refresh_functions = []

        def __refresh():
            for refresh_func in self.refresh_functions:
                refresh_func()

        # Setup refresh Timer
        self.refresh_timer = QtCore.QTimer()
        self.refresh_timer.setTimerType(QtCore.Qt.TimerType.PreciseTimer)
        self.refresh_timer.timeout.connect(__refresh)
        self.refresh_timer.start(refresh_interval_ms)  # Refresh at 200hz

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

        self.dock_area.addDock(field_dock, "left")
        self.dock_area.addDock(parameter_dock, "left", field_dock)
        self.dock_area.addDock(log_dock, "bottom", field_dock)
        self.dock_area.addDock(performance_dock, "right", log_dock)

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
        field_dock = Dock("Field", size=(500, 2000))
        field_dock.addWidget(self.field)

        return field_dock

    def setup_parameter_widget(self):
        """
        TODO
        """

        dict_obj = message_to_dict(ThunderbotsConfig())
        print(dict_obj)
        
        params = [
            {'name': 'Config', 'type': 'group', 'children': [
                    {'name': 'run_ai', 'type': 'bool', 'value': True},
                    {'name': 'threshold', 'type': 'slider', 'limits':[0,100], 'value': 0.0},
            ]},
        ]
        params2 = [
            {'name': 'Config', 'type': 'group', 'children': [
                    {'name': 'run_ai', 'type': 'bool', 'value': True},
            ]},
        ]
        
        
        # Create tree of Parameter objects
        p = Parameter.create(name='params', type='group', children=params)


        t = ParameterTree()
        t.setParameters(p, showTop=False)

        ## If anything changes in the tree, print a message
        def change(param, changes):
            print("tree changes:")
            for param, change, data in changes:
                path = p.childPath(param)
                if path is not None:
                    childName = '.'.join(path)
                else:
                    childName = param.name()
                print('  parameter: %s'% childName)
                print('  change:    %s'% change)
                print('  data:      %s'% str(data))
                print('  ----------')
            
        p.sigTreeStateChanged.connect(change)

        def valueChanging(param, value):
            print("Value changing (not finalized): %s %s" % (param, value))
        
        # Too lazy for recursion:
        for child in p.children():
            child.sigValueChanging.connect(valueChanging)
            for ch2 in child.children():
                ch2.sigValueChanging.connect(valueChanging)

        win = QtGui.QWidget()
        layout = QtGui.QVBoxLayout()
        win.setLayout(layout)
        layout.addWidget(t)

        # Create and return dock
        param_dock = Dock("Parameters", size=(20, 1000))
        param_dock.addWidget(win)

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
        log_dock = Dock("Logs", size=(500, 100))
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

    def show(self):
        self.window.show()
        pyqtgraph.exec()

    def close(self):
        QtCore.QTimer.singleShot(0, self.window.close)

if __name__ == '__main__':
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
