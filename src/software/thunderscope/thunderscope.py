import os
import time
import textwrap
import shelve
import signal
import platform
import logging

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

from qt_material import apply_stylesheet, list_themes

import pyqtgraph
import qdarktheme
from pyqtgraph.dockarea import *
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *

from software.py_constants import *
from proto.import_all_protos import *
from software.thunderscope.common.proto_plotter import ProtoPlotter
from extlibs.er_force_sim.src.protobuf.world_pb2 import *
from software.thunderscope.dock_label_style import *

# Import Widgets
from software.thunderscope.field import (
    obstacle_layer,
    path_layer,
    validation_layer,
    simulator_layer,
    world_layer,
    passing_layer,
    hrvo_layer,
)

from software.thunderscope.common.proto_configuration_widget import (
    ProtoConfigurationWidget,
)
from software.thunderscope.field.field import Field
from software.thunderscope.log.g3log_widget import g3logWidget
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.play.playinfo_widget import playInfoWidget
from software.thunderscope.robot_diagnostics.chicker_widget import ChickerWidget
from software.thunderscope.robot_diagnostics.diagnostics_input_widget import (
    FullSystemConnectWidget,
)
from software.thunderscope.robot_diagnostics.drive_and_dribbler_widget import (
    DriveAndDribblerWidget,
)
from software.thunderscope.robot_diagnostics.robot_view import RobotView
from software.thunderscope.robot_diagnostics.estop_view import EstopView
from software.thunderscope.replay.proto_player import ProtoPlayer

SAVED_LAYOUT_PATH = "/opt/tbotspython/saved_tscope_layout"
GAME_CONTROLLER_URL = "http://localhost:8081"


class Thunderscope(object):

    """ Thunderscope is our main visualizer that can visualize our field,
    obstacles, paths, performance metrics, logs, plots. Thunderscope also
    provides tools to interact with the robots.

    Thunderscope uses pyqtgraph, which is highly configurable during runtime.
    Users can move docks (purple bar) around, double click to pop them out into
    another window, etc. https://pyqtgraph.readthedocs.io/en/latest/

    The setup_* functions return docks. See configure_full_system_layout for an
    example. The returned docks can be arranged differently based on the
    use case (robot diagnostics, simulation, robocup, demo, etc..)

    """

    def __init__(
        self,
        simulator_proto_unix_io=None,
        blue_full_system_proto_unix_io=None,
        yellow_full_system_proto_unix_io=None,
        layout_path=None,
        load_blue=False,
        load_yellow=False,
        load_diagnostics=False,
        load_gamecontroller=True,
        blue_replay_log=None,
        yellow_replay_log=None,
        refresh_interval_ms=10,
        visualization_buffer_size=5,
    ):
        """Initialize Thunderscope

        :param simulator_proto_unix_io: The simulator's proto unix io
        :param blue_full_system_proto_unix_io: The blue full system's proto unix io
        :param yellow_full_system_proto_unix_io: The yellow full system's proto unix io
        :param layout_path: The path to the layout to load
        :param load_blue: Whether to load the blue dock area
        :param load_yellow: Whether to load the yellow dock area
        :param load_diagnostics: Whether to load the diagnostics dock area
        :param load_gamecontroller: Whether to load the gamecontroller window
        :param blue_replay_log: The blue replay log
        :param yellow_replay_log: The yellow replay log
        :param refresh_interval_ms:
            The interval in milliseconds to refresh all the widgets.
        :param visualization_buffer_size: The size of the visualization buffer.
            Increasing this will increase smoothness but will be less realtime. 

        """
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        # Setup MainApp and initialize DockArea
        self.app = pyqtgraph.mkQApp("Thunderscope")

        # Setup stylesheet
        apply_stylesheet(self.app, theme="dark_blue.xml")

        self.blue_replay_log = blue_replay_log
        self.yellow_replay_log = yellow_replay_log
        self.refresh_interval_ms = refresh_interval_ms
        self.visualization_buffer_size = visualization_buffer_size
        self.widgets = {}
        self.refresh_timers = []

        # TODO (#2586) Improve this layout
        self.tabs = QTabWidget()
        self.blue_full_system_dock_area = DockArea()
        self.yellow_full_system_dock_area = DockArea()
        self.robot_diagnostics_dock_area = DockArea()

        self.web_view = QWebEngineView()
        self.web_view.load(QtCore.QUrl(GAME_CONTROLLER_URL))

        if load_blue:
            self.tabs.addTab(self.blue_full_system_dock_area, "Blue Fullsystem")
        if load_yellow:
            self.tabs.addTab(self.yellow_full_system_dock_area, "Yellow Fullsystem")
        if load_diagnostics:
            self.tabs.addTab(self.robot_diagnostics_dock_area, "Robot Diagnostics")
        if load_gamecontroller:
            self.tabs.addTab(self.web_view, "Gamecontroller")

        self.window = QtGui.QMainWindow()
        self.window.setCentralWidget(self.tabs)
        self.window.setWindowTitle("Thunderscope")

        # ProtoUnixIOs
        #
        # NOTE: Simulated tests need to be able to run without Thunderscope
        # enabled, so the test fixture creates its own ProtoUnixIOs. But, we
        # would optionally like to enable thunderscope, observe protos and plot
        # them in thunderscope. So we need to switch over to the dependency
        # injected ProtoUnixIOs when provided.
        #
        # Also NOTE: We have two separate IOs for each full system because the
        # er force simulator expects two inputs of the same protobuf type but
        # from the blue or yellow team. We also would like to visualize the same
        # protobuf types on two separate widgets.
        #

        if load_blue:
            self.blue_full_system_proto_unix_io = (
                ProtoUnixIO()
                if blue_full_system_proto_unix_io is None
                else blue_full_system_proto_unix_io
            )
        if load_yellow:
            self.yellow_full_system_proto_unix_io = (
                ProtoUnixIO()
                if yellow_full_system_proto_unix_io is None
                else yellow_full_system_proto_unix_io
            )

        # the proto unix io to which diagnostics protos should be sent to
        # if one of the fullsystems is running, uses the same proto
        # if not, initialises a new one
        # only used if diagnostics is enabled
        self.robot_diagnostics_proto_unix_io = (
            self.blue_full_system_proto_unix_io
            if load_blue
            else self.yellow_full_system_proto_unix_io
            if load_yellow
            else ProtoUnixIO()
        )

        self.simulator_proto_unix_io = (
            ProtoUnixIO()
            if simulator_proto_unix_io is None
            else simulator_proto_unix_io
        )

        # Setup the main window and load the requested tabs
        self.configure_layout(layout_path, load_blue, load_yellow, load_diagnostics)

        # Save and Load Prompts
        #
        # NOTE: As long as Thunderscope has focus, the keyboard shortcuts will
        # work because they are setup on self.window.
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
        self.reset_layout_shortcut.activated.connect(self.reset_layout)

        self.show_help = QtGui.QShortcut(QtGui.QKeySequence("h"), self.window)
        self.show_help.activated.connect(
            lambda: QMessageBox.information(
                self.window,
                "Help",
                textwrap.dedent(
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
                    """
                ),
            )
        )

    def reset_layout(self):
        """Reset the layout to the default layout"""

        if os.path.exists(SAVED_LAYOUT_PATH):
            os.remove(SAVED_LAYOUT_PATH)
            QMessageBox.information(
                self.window,
                "Restart Required",
                "Restart thunderscope to reset the layout.",
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
            logging.warn("No filename selected")
            return

        with shelve.open(filename, "c") as shelf:
            shelf["blue_dock_state"] = self.blue_full_system_dock_area.saveState()
            shelf["yellow_dock_state"] = self.yellow_full_system_dock_area.saveState()
            shelf[
                "robot_diagnostics_dock_state"
            ] = self.robot_diagnostics_dock_area.saveState()

        with shelve.open(SAVED_LAYOUT_PATH, "c") as shelf:
            shelf["blue_dock_state"] = self.blue_full_system_dock_area.saveState()
            shelf["yellow_dock_state"] = self.yellow_full_system_dock_area.saveState()
            shelf[
                "robot_diagnostics_dock_state"
            ] = self.robot_diagnostics_dock_area.saveState()

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
                logging.warn("No filename selected")
                return

        # lets load the layouts from the shelf into their respective dock areas
        # if the dock doesn't exist in the default layout, we ignore it
        # (instead of adding a placeholder dock)
        with shelve.open(filename, "r") as shelf:

            self.blue_full_system_dock_area.restoreState(
                shelf["blue_dock_state"], missing="ignore"
            )
            self.yellow_full_system_dock_area.restoreState(
                shelf["yellow_dock_state"], missing="ignore"
            )
            self.robot_diagnostics_dock_area.restoreState(
                shelf["robot_diagnostics_dock_state"], missing="ignore"
            )

            # Update default layout
            if filename != SAVED_LAYOUT_PATH:
                with shelve.open(SAVED_LAYOUT_PATH, "c") as default_shelf:
                    default_shelf["blue_dock_state"] = shelf["blue_dock_state"]
                    default_shelf["yellow_dock_state"] = shelf["yellow_dock_state"]
                    default_shelf["robot_diagnostics_dock_state"] = shelf[
                        "robot_diagnostics_dock_state"
                    ]
                    default_shelf.sync()

    def configure_layout(
        self, layout_path, load_blue=False, load_yellow=False, load_diagnostics=False
    ):
        """Load the specified layout or the default file. If the default layout
        file doesn't exist, and no layout is provided, then just configure
        the default layout.

        :param layout_path: Path to the layout file to load.
        :param load_blue: Whether to load the blue layout.
        :param load_yellow: Whether to load the yellow layout.
        :param load_diagnostics: Whether to load the diagnostics layout.
        """

        # whether the fullsystem tab should have the robot view widget
        load_fullsystem_robot_view = True

        if load_blue == load_yellow:
            # in AI vs AI mode, fullsystem tab should not have robot view
            load_fullsystem_robot_view = False

        if load_yellow:
            self.configure_full_system_layout(
                self.yellow_full_system_dock_area,
                self.simulator_proto_unix_io,
                self.yellow_full_system_proto_unix_io,
                load_fullsystem_robot_view,
                load_diagnostics,
                True,
            )
        if load_blue:
            self.configure_full_system_layout(
                self.blue_full_system_dock_area,
                self.simulator_proto_unix_io,
                self.blue_full_system_proto_unix_io,
                load_fullsystem_robot_view,
                load_diagnostics,
                False,
            )

        if load_yellow or load_blue:
            path = layout_path if layout_path else SAVED_LAYOUT_PATH

            try:
                self.load_layout(path)
            except Exception:
                pass

        if load_diagnostics:
            self.configure_robot_diagnostics_layout(
                self.robot_diagnostics_dock_area,
                self.robot_diagnostics_proto_unix_io,
                load_blue or load_yellow,
            )

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

    def configure_full_system_layout(
        self,
        dock_area,
        sim_proto_unix_io,
        full_system_proto_unix_io,
        load_robot_view,
        load_diagnostics,
        friendly_colour_yellow,
    ):
        """Configure the default layout for thunderscope

        :param dock_area: The dock area to configure the layout
        :param sim_proto_unix_io: The proto unix io object for the simulator
        :param full_system_proto_unix_io: The proto unix io object for the full system
        :param load_robot_view: Whether robot view should be loaded on the fullsystem tab or not
                                - should not be loaded in AI vs AI
        :param load_diagnostics: Whether diagnostics is being loaded currently
                                 - robot view should have checkboxes if diagnostics is loaded
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

        widgets["parameter_widget"] = self.setup_parameter_widget(
            full_system_proto_unix_io, friendly_colour_yellow
        )
        parameter_dock = Dock("Parameters")
        parameter_dock.addWidget(widgets["parameter_widget"])

        widgets["playinfo_widget"] = self.setup_play_info(full_system_proto_unix_io)
        playinfo_dock = Dock("Play Info")
        playinfo_dock.addWidget(widgets["playinfo_widget"])

        dock_area.addDock(field_dock)
        dock_area.addDock(log_dock, "left", field_dock)
        dock_area.addDock(parameter_dock, "above", log_dock)
        dock_area.addDock(playinfo_dock, "bottom", field_dock)
        dock_area.addDock(performance_dock, "right", playinfo_dock)

        if load_robot_view:
            widgets["robot_view"] = self.setup_robot_view(
                full_system_proto_unix_io, load_diagnostics
            )
            robot_view_dock = Dock("RobotView")
            robot_view_dock.addWidget(widgets["robot_view"])
            dock_area.addDock(robot_view_dock, "above", log_dock)
            if load_diagnostics:
                self.toggle_robot_connection_signal = widgets["robot_view"].toggle_robot_connection_signal

    def configure_robot_diagnostics_layout(
        self, dock_area, proto_unix_io, load_fullsystem,
    ):
        """Configure the default layout for the robot diagnostics widget

        :param dock_area: The dock area to configure the layout
        :param proto_unix_io: The proto unix io object for the full system
        :param load_fullsystem: Whether the fullsystem is also being loaded currently
        """

        self.diagnostics_widgets = {}

        self.diagnostics_widgets["chicker"] = self.setup_chicker_widget(proto_unix_io)
        chicker_dock = Dock("Chicker")
        chicker_dock.addWidget(self.diagnostics_widgets["chicker"])

        self.diagnostics_widgets["drive"] = self.setup_drive_and_dribbler_widget(
            proto_unix_io
        )
        drive_dock = Dock("Drive and Dribbler")
        drive_dock.addWidget(self.diagnostics_widgets["drive"])

        self.diagnostics_widgets["log_widget"] = self.setup_log_widget(proto_unix_io)
        log_dock = Dock("Logs")
        log_dock.addWidget(self.diagnostics_widgets["log_widget"])

        if not load_fullsystem:
            self.diagnostics_widgets["robot_view"] = self.setup_robot_view(
                proto_unix_io, True
            )
            robot_view_dock = Dock("RobotView")
            robot_view_dock.addWidget(self.diagnostics_widgets["robot_view"])
            self.toggle_robot_connection_signal = self.diagnostics_widgets[
                "robot_view"
            ].toggle_robot_connection_signal

        self.diagnostics_widgets[
            "diagnostics_input"
        ] = self.setup_diagnostics_input_widget(proto_unix_io)
        diagnostics_input_dock = Dock("Diagnostics_Input")
        diagnostics_input_dock.addWidget(self.diagnostics_widgets["diagnostics_input"])

        self.diagnostics_widgets["diagnostics_input"].toggle_controls_signal.connect(
            self.diagnostics_widgets["drive"].toggle_all
        )

        self.diagnostics_widgets["diagnostics_input"].toggle_controls_signal.connect(
            self.diagnostics_widgets["chicker"].set_should_enable_buttons
        )

        self.robot_diagnostics_dock_area.addDock(log_dock)
        if not load_fullsystem:
            dock_area.addDock(robot_view_dock, "above", log_dock)
        self.robot_diagnostics_dock_area.addDock(drive_dock, "right", log_dock)
        self.robot_diagnostics_dock_area.addDock(chicker_dock, "below", drive_dock)
        self.robot_diagnostics_dock_area.addDock(
            diagnostics_input_dock, "top", chicker_dock
        )

        estop_view = self.setup_estop_view(proto_unix_io)

        dock = Dock("Estop View")
        dock.addWidget(estop_view)
        self.robot_diagnostics_dock_area.addDock(dock, "bottom", log_dock)

    def setup_robot_view(self, proto_unix_io, load_diagnostics):
        """Setup the robot view widget
        :param proto_unix_io: The proto unix io object for the full system
        :param load_diagnostics: Boolean to indicate if robot diagnostics should be loaded
        """
        robot_view = RobotView(load_diagnostics)
        self.register_refresh_function(robot_view.refresh)
        proto_unix_io.register_observer(RobotStatus, robot_view.robot_status_buffer)
        return robot_view

    def setup_estop_view(self, proto_unix_io):
        """Setup the estop view widget

        :param proto_unix_io: The proto unix io object for the full system

        """
        estop_view = EstopView()
        self.register_refresh_function(estop_view.refresh)

        proto_unix_io.register_observer(EstopState, estop_view.estop_state_buffer)
        return estop_view

    def setup_field_widget(
        self, sim_proto_unix_io, full_system_proto_unix_io, friendly_colour_yellow
    ):
        """setup the field widget with the constituent layers

        :param sim_proto_unix_io: The proto unix io object for the simulator
        :param full_system_proto_unix_io: The proto unix io object for the full system
        :param friendly_colour_yellow: Whether the friendly colour is yellow
        :returns: the field widget

        """
        self.player = ProtoPlayer(
            self.yellow_replay_log if friendly_colour_yellow else self.blue_replay_log,
            full_system_proto_unix_io,
        )
        field = Field(player=self.player)

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
        hrvo_sim_states = []
        # Add HRVO layers to field widget and have them hidden on startup
        # TODO (#2655): Add/Remove HRVO layers dynamically based on the HRVOVisualization proto messages
        for robot_id in range(MAX_ROBOT_IDS_PER_SIDE):
            hrvo_sim_state = hrvo_layer.HRVOLayer(
                robot_id, self.visualization_buffer_size
            )
            hrvo_sim_states.append(hrvo_sim_state)
            field.add_layer(f"HRVO {robot_id}", hrvo_sim_state, False)

        # Register observers
        sim_proto_unix_io.register_observer(
            SimulatorState, sim_state.simulator_state_buffer
        )

        for arg in [
            (World, world.world_buffer),
            (RobotStatus, world.robot_status_buffer),
            (Referee, world.referee_buffer),
            (PrimitiveSet, obstacles.primitive_set_buffer),
            (PrimitiveSet, paths.primitive_set_buffer),
            (PassVisualization, passing.pass_visualization_buffer),
            (ValidationProtoSet, validation.validation_set_buffer),
            (SimulatorState, sim_state.simulator_state_buffer),
        ] + [
            (HRVOVisualization, hrvo_sim_state.hrvo_buffer)
            for hrvo_sim_state in hrvo_sim_states
        ]:
            full_system_proto_unix_io.register_observer(*arg)

        # Register refresh functions
        self.register_refresh_function(field.refresh)

        return field

    def setup_parameter_widget(self, proto_unix_io, friendly_colour_yellow):
        """Setup the parameter widget

        :param proto_unix_io: The proto unix io object
        :param friendly_colour_yellow: 
        :returns: The proto configuration widget

        """

        self.config = ThunderbotsConfig()
        self.config.sensor_fusion_config.friendly_color_yellow = friendly_colour_yellow

        def on_change_callback(attr, value, updated_proto):
            proto_unix_io.send_proto(ThunderbotsConfig, updated_proto)

        return ProtoConfigurationWidget(self.config, on_change_callback)

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

        def extract_namedvalue_data(named_value_data):
            return {named_value_data.name: named_value_data.value}

        # Performance Plots plot HZ so the values can't be negative
        proto_plotter = ProtoPlotter(
            min_y=0,
            max_y=100,
            window_secs=15,
            configuration={NamedValue: extract_namedvalue_data},
        )

        # Register observer
        proto_unix_io.register_observer(NamedValue, proto_plotter.buffers[NamedValue])
        # Register refresh function
        self.register_refresh_function(proto_plotter.refresh)
        return proto_plotter

    def setup_play_info(self, proto_unix_io):
        """Setup the play info widget

        :param proto_unix_io: The proto unix io object
        :returns: The play info widget

        """

        play_info = playInfoWidget()
        proto_unix_io.register_observer(PlayInfo, play_info.playinfo_buffer)
        proto_unix_io.register_observer(Referee, play_info.referee_buffer)
        self.register_refresh_function(play_info.refresh)

        return play_info

    def setup_chicker_widget(self, proto_unix_io):
        """Setup the chicker widget for robot diagnostics

        :param proto_unix_io: The proto unix io object
        :returns: The chicker widget

        """
        # Create widget
        chicker_widget = ChickerWidget(proto_unix_io)

        # Register refresh function
        self.register_refresh_function(chicker_widget.refresh)

        return chicker_widget

    def setup_diagnostics_input_widget(self, proto_unix_io):
        """

        Sets up the diagnostics input widget with the given proto unix io

        :param proto_unix_io: The proto unix io object
        :returns the fullsystem connect widget
        """

        diagnostics_input_widget = FullSystemConnectWidget(proto_unix_io)

        self.register_refresh_function(diagnostics_input_widget.refresh)

        return diagnostics_input_widget

    def setup_drive_and_dribbler_widget(self, proto_unix_io):
        """Setup the drive and dribbler widget

        :param proto_unix_io: The proto unix io object
        :returns: The drive and dribbler widget

        """
        drive_and_dribbler_widget = DriveAndDribblerWidget(proto_unix_io)
        self.register_refresh_function(drive_and_dribbler_widget.refresh)

        return drive_and_dribbler_widget

    def show(self):
        """Show the main window"""

        self.window.show()
        pyqtgraph.exec()

    def close(self):
        """Close the main window"""

        QtCore.QTimer.singleShot(0, self.window.close)
