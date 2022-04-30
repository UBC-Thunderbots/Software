import os
import time
import shelve
import signal
import platform
import numpy

# PyQt5 doesn't play nicely with i3 and Ubuntu 18, PyQt6 is much more stable
# Unfortunately, PyQt6 doesn't install on Ubuntu 18. Thankfully both
# libraries are interchangeable, and  we just need to swap them in this
# one spot, and pyqtgraph will pick up on it and store the library under
# pyqtgraph.Qt. So from PyQt5 import x becomes from pyqtgraph.Qt import x
if "18.04" in platform.version():
    import PyQt5
    from PyQt5.QtWebEngineWidgets import QWebEngineView
else:
    import PyQt6
    from PyQt6.QtWebEngineWidgets import QWebEngineView

import qdarktheme

import pyqtgraph
from pyqtgraph.dockarea import *
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *

import software.python_bindings as cpp_bindings

from proto.import_all_protos import *
from proto.message_translation import tbots_protobuf

from software.py_constants import *
from software.networking import threaded_unix_sender
from software.thunderscope.robot_communication import RobotCommunication
from software.thunderscope.arbitrary_plot.named_value_plotter import NamedValuePlotter
from software.thunderscope.binary_context_managers import *
from extlibs.er_force_sim.src.protobuf.world_pb2 import *

# Import Widgets
from software.thunderscope.field import (
    obstacle_layer,
    path_layer,
    validation_layer,
    simulator_layer,
    world_layer,
    passing_layer,
)

from software.thunderscope.field.field import Field
from software.thunderscope.log.g3log_widget import g3logWidget
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.play.playinfo_widget import playInfoWidget
from software.thunderscope.robot_diagnostics.chicker import ChickerWidget
from software.thunderscope.robot_diagnostics.drive_and_dribbler_widget import (
    DriveAndDribblerWidget,
)

SAVED_LAYOUT_PATH = "/opt/tbotspython/saved_tscope_layout"
NUM_ROBOTS = 6
SIM_TICK_RATE_MS = 16
REFRESH_INTERVAL_MS = 5
GAME_CONTROLLER_URL = "http://localhost:8081"


class Thunderscope(object):

    """ Thunderscope is our main visualizer that can visualize our field,
    obstacles, paths, performance metrics, logs, plots. Thunderscope also
    provides tools to interact with the robots.

    Thunderscope uses pyqtgraph, which is highly configurable during runtime.
    Users can move docks (purple bar) around, double click to pop them out into
    another window, etc. https://pyqtgraph.readthedocs.io/en/latest/

    The setup_* functions return docks. See configure_default_layout for an
    example. The returned docks can be arranged differently based on the
    use case (robot diagnostics, simulation, robocup, demo, etc..)

    """

    def __init__(
        self,
        simulator_proto_unix_io=None,
        blue_full_system_proto_unix_io=None,
        yellow_full_system_proto_unix_io=None,
        refresh_interval_ms=10,
        visualization_buffer_size=5,
    ):
        """Initialize Thunderscope

        :param simulator_proto_unix_io: The simulator's proto unix io
        :param blue_full_system_proto_unix_io: The blue full system's proto unix io
        :param yellow_full_system_proto_unix_io: The yellow full system's proto unix io
        :param refresh_interval_ms: The interval in milliseconds to refresh the simulator
        :param visualization_buffer_size: The size of the visualization buffer

        """

        # Setup MainApp and initialize DockArea
        self.app = pyqtgraph.mkQApp("Thunderscope")
        self.app.setStyleSheet(qdarktheme.load_stylesheet())
        self.refresh_interval_ms = refresh_interval_ms
        self.visualization_buffer_size = visualization_buffer_size
        self.widgets = {}

        signal.signal(signal.SIGINT, signal.SIG_DFL)

        # TODO (#2586) Improve this layout
        self.tabs = QTabWidget()
        self.blue_full_system_dock_area = DockArea()
        self.yellow_full_system_dock_area = DockArea()

        self.web_view = QWebEngineView()
        self.web_view.load(QtCore.QUrl(GAME_CONTROLLER_URL))

        self.tabs.addTab(self.blue_full_system_dock_area, "Blue Fullsystem")
        self.tabs.addTab(self.yellow_full_system_dock_area, "Yellow Fullsystem")
        self.tabs.addTab(self.web_view, "Gamecontroller")

        self.window = QtGui.QMainWindow()
        self.window.setCentralWidget(self.tabs)
        self.window.setWindowTitle("Thunderscope")

        # ProtoUnixIOs
        #
        # NOTE: We have two separate IOs for each full system because the
        # er force simulator expects two inputs of the same protobuf type but
        # from the blue or yellow team. We also would like to visualize the same
        # protobuf types on two separate widgets.
        self.simulator_proto_unix_io = (
            ProtoUnixIO()
            if simulator_proto_unix_io is None
            else simulator_proto_unix_io
        )
        self.yellow_full_system_proto_unix_io = (
            ProtoUnixIO()
            if yellow_full_system_proto_unix_io is None
            else yellow_full_system_proto_unix_io
        )
        self.blue_full_system_proto_unix_io = (
            ProtoUnixIO()
            if blue_full_system_proto_unix_io is None
            else blue_full_system_proto_unix_io
        )

        self.refresh_timers = []

        # Save and Load Prompts
        self.save_layout_shortcut = QtGui.QShortcut(
            QtGui.QKeySequence("Ctrl+S"), self.window
        )
        self.save_layout_shortcut.activated.connect(self.save_layout)

        self.open_layout_shortcut = QtGui.QShortcut(
            QtGui.QKeySequence("Ctrl+O"), self.window
        )
        self.open_layout_shortcut.activated.connect(self.load_layout)

        self.reset_layout_shortcut = QtGui.QShortcut(
            QtGui.QKeySequence("Ctrl+R"), self.window
        )

        def __reset_layout():
            if os.path.exists(SAVED_LAYOUT_PATH):
                os.remove(SAVED_LAYOUT_PATH)
                QMessageBox.information(
                    self.window,
                    "Restart Required",
                    """
                    Restart thunderscope to reset the layout.
                    """,
                )

        self.reset_layout_shortcut.activated.connect(__reset_layout)

        self.show_help = QtGui.QShortcut(QtGui.QKeySequence("h"), self.window)
        self.show_help.activated.connect(
            lambda: QMessageBox.information(
                self.window,
                "Help",
                f"""
Keyboard Shortcuts:

I to identify robots, show their IDs
Cntrl+S: Save Layout
Cntrl+O: Open Layout
Cntrl+R: will remove the file and reset the layout

Layout file (on save) is located at 
        {SAVED_LAYOUT_PATH}

Mouse Shortcuts:

Double Click Purple Bar to pop window out
Drag Purple Bar to rearrange docks
Click items in legends to select/deselect
Cntrl-Click and Drag: Move ball and kick
""",
            )
        )

    def save_layout(self):
        """Open a file dialog to save the layout and any other
        registered state to a file

        """

        filename, _ = QtGui.QFileDialog.getSaveFileName(
            self.window,
            "Save layout",
            "~/dock_layout_{}.tscopelayout".format(int(time.time())),
            options=QFileDialog.Option.DontUseNativeDialog,
        )

        if not filename:
            print("No filename selected")
            return

        with shelve.open(filename, "c") as shelf:
            shelf["blue_dock_state"] = self.blue_full_system_dock_area.saveState()
            shelf["yellow_dock_state"] = self.yellow_full_system_dock_area.saveState()

        with shelve.open(SAVED_LAYOUT_PATH, "c") as shelf:
            shelf["blue_dock_state"] = self.blue_full_system_dock_area.saveState()
            shelf["yellow_dock_state"] = self.yellow_full_system_dock_area.saveState()

    def load_layout(self, filename=None):
        """Open a file dialog to load the layout and state to all widgets

        :param filename: The filename to load the layout from. If None, then
                         open a file dialog.

        """

        if filename is None:
            filename, _ = QtGui.QFileDialog.getOpenFileName(
                self.window,
                "Open layout",
                "~/",
                options=QFileDialog.Option.DontUseNativeDialog,
            )

            if not filename:
                print("No filename selected")
                return

        with shelve.open(filename, "r") as shelf:
            self.blue_full_system_dock_area.restoreState(
                shelf["blue_dock_state"], missing="ignore"
            )
            self.yellow_full_system_dock_area.restoreState(
                shelf["yellow_dock_state"], missing="ignore"
            )

            # Update default layout
            if filename != SAVED_LAYOUT_PATH:
                with shelve.open(SAVED_LAYOUT_PATH, "c") as default_shelf:
                    default_shelf["blue_dock_state"] = shelf["blue_dock_state"]
                    default_shelf["yellow_dock_state"] = shelf["yellow_dock_state"]
                    default_shelf.sync()

    def load_saved_layout(self, layout_path, load_blue=True, load_yellow=True):
        """Load the specified layout or the default file. If the default layout
        file doesn't exist, and no layout is provided, then just configure
        the default layout.

        :param layout_path: Path to the layout file to load.
        :param load_blue: Whether to load the blue layout.
        :param load_yellow: Whether to load the yellow layout.

        """
        if load_yellow:
            self.configure_default_layout(
                self.yellow_full_system_dock_area,
                self.simulator_proto_unix_io,
                self.yellow_full_system_proto_unix_io,
                True,
            )

        if load_blue:
            self.configure_default_layout(
                self.blue_full_system_dock_area,
                self.simulator_proto_unix_io,
                self.blue_full_system_proto_unix_io,
                False,
            )

        path = layout_path if layout_path else SAVED_LAYOUT_PATH

        try:
            self.load_layout(path)
        except Exception:
            pass

    def register_refresh_function(self, refresh_func):
        """Register the refresh functions to run at the refresh_interval_ms
        passed into thunderscope.

        :param refresh_func: The function to call at refresh_interval_ms

        """

        refresh_timer = QtCore.QTimer()
        refresh_timer.setTimerType(QtCore.Qt.TimerType.PreciseTimer)
        refresh_timer.timeout.connect(refresh_func)
        refresh_timer.start(self.refresh_interval_ms)

        self.refresh_timers.append(refresh_timer)

    def configure_default_layout(
        self,
        dock_area,
        sim_proto_unix_io,
        full_system_proto_unix_io,
        friendly_colour_yellow,
    ):
        """Configure the default layout for thunderscope

        :param dock_area: The dock area to configure the layout
        :param sim_proto_unix_io: The proto unix io object for the simulator
        :param full_system_proto_unix_io: The proto unix io object for the full system
        :param friendly_colour_yellow: Whether the friendly colour is yellow

        """
        # Configure Docks and save them
        self.widgets[friendly_colour_yellow] = {}
        widgets = self.widgets[friendly_colour_yellow]

        widgets["field_widget"] = self.setup_field_widget(
            sim_proto_unix_io, full_system_proto_unix_io, friendly_colour_yellow
        )
        field_dock = Dock("Field")
        field_dock.addWidget(widgets["field_widget"])

        widgets["log_widget"] = self.setup_log_widget(full_system_proto_unix_io)
        log_dock = Dock("Logs")
        log_dock.addWidget(widgets["log_widget"])

        widgets["performance_widget"] = self.setup_performance_plot(
            full_system_proto_unix_io
        )
        performance_dock = Dock("Performance")
        performance_dock.addWidget(widgets["performance_widget"].win)

        widgets["playinfo_widget"] = self.setup_play_info(full_system_proto_unix_io)
        playinfo_dock = Dock("Play Info")
        playinfo_dock.addWidget(widgets["playinfo_widget"])

        dock_area.addDock(field_dock)
        dock_area.addDock(log_dock, "bottom", field_dock)
        dock_area.addDock(performance_dock, "right", log_dock)
        dock_area.addDock(playinfo_dock, "right", performance_dock)

    def setup_field_widget(
        self, sim_proto_unix_io, full_system_proto_unix_io, friendly_colour_yellow
    ):
        """setup the field widget with the constituent layers

        :param sim_proto_unix_io: The proto unix io object for the simulator
        :param full_system_proto_unix_io: The proto unix io object for the full system
        :param friendly_colour_yellow: Whether the friendly colour is yellow
        :returns: the field widget

        """
        field = Field()

        # Create layers
        paths = path_layer.PathLayer(self.visualization_buffer_size)
        obstacles = obstacle_layer.ObstacleLayer(self.visualization_buffer_size)
        validation = validation_layer.ValidationLayer(self.visualization_buffer_size)
        world = world_layer.WorldLayer(
            sim_proto_unix_io, friendly_colour_yellow, self.visualization_buffer_size
        )
        sim_state = simulator_layer.SimulatorLayer(
            friendly_colour_yellow, self.visualization_buffer_size
        )
        passing = passing_layer.PassingLayer(self.visualization_buffer_size)

        # Add field layers to field
        field.add_layer("Vision", world)
        field.add_layer("Obstacles", obstacles)
        field.add_layer("Paths", paths)
        field.add_layer("Validation", validation)
        field.add_layer("Passing", passing)
        field.add_layer("Simulator", sim_state)

        # Register observers
        sim_proto_unix_io.register_observer(
            SimulatorState, sim_state.simulator_state_buffer
        )

        for arg in [
            (World, world.world_buffer),
            (RobotStatus, world.robot_status_buffer),
            (Referee, world.referee_buffer),
            (Obstacles, obstacles.obstacle_buffer),
            (PathVisualization, paths.path_visualization_buffer),
            (PassVisualization, passing.pass_visualization_buffer),
            (ValidationProtoSet, validation.validation_set_buffer),
            (SimulatorState, sim_state.simulator_state_buffer),
        ]:
            full_system_proto_unix_io.register_observer(*arg)

        # Register refresh functions
        self.register_refresh_function(field.refresh)

        return field

    def setup_log_widget(self, proto_unix_io):
        """Setup the wiget that receives logs from full system

        :param proto_unix_io: The proto unix io object
        :returns: The log widget

        """
        # Create widget
        logs = g3logWidget()

        # Register observer
        proto_unix_io.register_observer(RobotLog, logs.log_buffer)

        # Register refresh function
        self.register_refresh_function(logs.refresh)

        return logs

    def setup_performance_plot(self, proto_unix_io):
        """Setup the performance plot

        :param proto_unix_io: The proto unix io object
        :returns: The performance plot widget

        """
        # Create widget
        named_value_plotter = NamedValuePlotter()

        # Register observer
        proto_unix_io.register_observer(
            NamedValue, named_value_plotter.named_value_buffer
        )

        # Register refresh function
        self.register_refresh_function(named_value_plotter.refresh)

        return named_value_plotter

    def setup_play_info(self, proto_unix_io):
        """Setup the play info widget

        :param proto_unix_io: The proto unix io object
        :returns: The play info widget

        """

        play_info = playInfoWidget()
        proto_unix_io.register_observer(PlayInfo, play_info.playinfo_buffer)
        self.register_refresh_function(play_info.refresh)

        return play_info

    def setup_chicker_widget(self, proto_unix_io):
        """Setup the chicker widget for robot diagnostics

        :param proto_unix_io: The proto unix io object
        :returns: The chicker widget

        """
        # Create widget
        chicker_widget = ChickerWidget()

        # Register refresh function
        self.register_refresh_function(self.chicker_widget.refresh)

        return chicker_widget

    def setup_drive_and_dribbler_widget(self):
        """Setup the drive and dribbler widget

        :returns: The drive and dribbler widget

        """

        return DriveAndDribblerWidget()

    def show(self):
        """Show the main window"""

        self.window.show()
        pyqtgraph.exec()

    def close(self):
        QtCore.QTimer.singleShot(0, self.window.close)
