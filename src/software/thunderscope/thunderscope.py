import os
import atexit
import signal
import threading
import time
import shelve
import signal
import time
import shelve
import argparse
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
import qdarktheme
from pyqtgraph.dockarea import *
from pyqtgraph.parametertree import Parameter, ParameterTree
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *

from proto.import_all_protos import *
from proto.message_translation import tbots_protobuf

import software.python_bindings as geom
from software.py_constants import *
from software.networking import threaded_unix_sender, networking
from software.thunderscope.robot_communication import RobotCommunication
from software.thunderscope.arbitrary_plot.named_value_plotter import NamedValuePlotter
from software.thunderscope.common.proto_configuration_widget import (
    ProtoConfigurationWidget,
)
from software.thunderscope.binary_context_managers import (
    FullSystem,
    Simulator,
    Gamecontroller,
)
from extlibs.er_force_sim.src.protobuf.world_pb2 import (
    SimulatorState,
    SimBall,
    SimRobot,
)

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
    ):
        """Initialize Thunderscope

        :param simulator_proto_unix_io: The simulator's proto unix io
        :param blue_full_system_proto_unix_io: The blue full system's proto unix io
        :param yellow_full_system_proto_unix_io: The blue full system's proto unix io
        :param refresh_interval_ms: The interval in milliseconds to refresh the simulator

        """
        pyqtgraph.setConfigOption("antialias", True)

        # Setup MainApp and initialize DockArea
        self.app = pyqtgraph.mkQApp("Thunderscope")
        self.app.setStyleSheet(qdarktheme.load_stylesheet())
        self.refresh_interval_ms = refresh_interval_ms
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
                "\nKeyboard Shortcuts:\n\n"
                "Cntrl+S: Save Layout\n"
                "Cntrl+O: Open Layout\n"
                "Cntrl+R: will remove the file and reset the layout\n"
                f"\nLayout file (on save) is located at \n{SAVED_LAYOUT_PATH}\n"
                "I to identify robots, show their IDs\n"
                "\nMouse Shortcuts:\n\n"
                "Double Click Purple Bar to pop window out\n"
                "Drag Purple Bar to rearrange docks\n"
                "Click items in legends to select/deselect\n"
                "Cntrl-Click and Drag: Move ball and kick\n",
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

        except Exception as exc:
            print(
                exc,
                Warning(
                    "No layout file specified and default "
                    + "layout at {} doesn't exist".format(path)
                ),
            )

        def __save_layout():

            filename, _ = QtGui.QFileDialog.getSaveFileName(
                self.window,
                "Save layout",
                "~/dock_layout_{}.tscopelayout".format(int(time.time())),
                options=QFileDialog.Option.DontUseNativeDialog,
            )

            result = self.dock_area.saveState()

            with shelve.open(filename, "c") as shelf:
                shelf["dock_state"] = result

        def __load_layout():

            filename, _ = QtGui.QFileDialog.getOpenFileName(
                self.window,
                "Open layout",
                "~/",
                options=QFileDialog.Option.DontUseNativeDialog,
            )

            with shelve.open(filename, "r") as shelf:
                self.dock_area.restoreState(shelf["dock_state"])

        self.save_layout = QtGui.QShortcut(QtGui.QKeySequence("Ctrl+S"), self.window)
        self.save_layout.activated.connect(__save_layout)

        self.save_layout = QtGui.QShortcut(QtGui.QKeySequence("Ctrl+O"), self.window)
        self.save_layout.activated.connect(__load_layout)

        self.show_help = QtGui.QShortcut(QtGui.QKeySequence("h"), self.window)
        self.show_help.activated.connect(
            lambda: QMessageBox.information(
                self.window,
                "Help",
                """
                Cntrl+S: Save Layout
                Double Click Purple Bar to pop window out
                Drag Purple Bar to rearrange docks
                Click items in legends to select/deselect
                """,
            )
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

        widgets["parameter_widget"] = self.setup_parameter_widget(
            full_system_proto_unix_io
        )
        parameter_dock = Dock("Parameters")
        parameter_dock.addWidget(widgets["parameter_widget"])

        widgets["playinfo_widget"] = self.setup_play_info(full_system_proto_unix_io)
        playinfo_dock = Dock("Play Info")
        playinfo_dock.addWidget(widgets["playinfo_widget"])

        dock_area.addDock(field_dock)
        dock_area.addDock(parameter_dock, "left", field_dock)
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
        paths = path_layer.PathLayer()
        obstacles = obstacle_layer.ObstacleLayer()
        validation = validation_layer.ValidationLayer()
        world = world_layer.WorldLayer(sim_proto_unix_io, friendly_colour_yellow)
        sim_state = simulator_layer.SimulatorLayer(friendly_colour_yellow)
        passing = passing_layer.PassingLayer()

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

    def setup_parameter_widget(self, proto_unix_io):
        def cak(p, t):
            print(p, t)

        # TODO run this up
        # p = ProtoConfigurationWidget(World, cak)

        return QtGui.QWidget()

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


if __name__ == "__main__":

    # Setup parser
    parser = argparse.ArgumentParser(
        description="Thunderscope: Run with no arguments to run AI vs AI"
    )

    parser.add_argument(
        "--layout",
        action="store",
        help="Which layout to run, if not specified the last layout will run",
    )

    # Runtime directories
    parser.add_argument(
        "--simulator_runtime_dir",
        type=str,
        help="simulator runtime directory",
        default="/tmp/tbots",
    )
    parser.add_argument(
        "--blue_fullsystem_runtime_dir",
        type=str,
        help="blue fullsystem runtime directory",
        default="/tmp/tbots/blue",
    )
    parser.add_argument(
        "--yellow_fullsystem_runtime_dir",
        type=str,
        help="yellow fullsystem runtime directory",
        default="/tmp/tbots/yellow",
    )

    # Debugging
    parser.add_argument(
        "--debug_fullsystem",
        action="store_true",
        default=False,
        help="Debug fullsystem",
    )
    parser.add_argument(
        "--debug_simulator",
        action="store_true",
        default=False,
        help="Debug the simulator",
    )
    parser.add_argument(
        "--visualize_cpp_test",
        action="store_true",
        default=False,
        help="Visualize C++ Tests",
    )

    # Run blue or yellow full system over WiFi
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--run_blue",
        action="store_true",
        help="Run full system as the blue team, over WiFi; estop required",
    )
    group.add_argument(
        "--run_yellow",
        action="store_true",
        help="Run full system as the yellow team, over WiFi; estop required",
    )
    parser.add_argument(
        "--interface",
        action="store",
        type=str,
        default=None,
        help="Which interface to communicate over",
    )

    # Sanity check that an interface was provided
    args = parser.parse_args()

    if args.run_blue or args.run_yellow:
        if args.interface is None:
            parser.error("Must specify interface")

    tscope = Thunderscope()

    # TODO (#2581) remove this
    if args.visualize_cpp_test:

        runtime_dir = "/tmp/tbots/yellow_test"

        try:
            os.mkdir("/tmp/tbots")
            os.mkdir(runtime_dir)
        except OSError:
            pass

        proto_unix_io = tscope.blue_full_system_proto_unix_io

        # Setup LOG(VISUALIZE) handling from full system. We set from_log_visualize
        # to true to decode from base64.
        for arg in [
            (runtime_dir, Obstacles, True),
            (runtime_dir, PathVisualization, True),
            (runtime_dir, PassVisualization, True),
            (runtime_dir, NamedValue, True),
            (runtime_dir, World, True),
            (runtime_dir, PlayInfo, True),
        ]:
            proto_unix_io.attach_unix_receiver(*arg)

        proto_unix_io.attach_unix_receiver(runtime_dir + "/log", RobotLog)

        tscope.load_saved_layout(args.layout, load_yellow=True)
        tscope.show()

    ###########################################################################
    #              AI + Robot Communication + Robot Diagnostics               #
    ###########################################################################
    #
    # When we are running with real robots. We want to run 1 instance of AI
    # and 1 instance of RobotCommunication which will send/recv packets over
    # the provided multicast channel.
    proto_unix_io = tscope.blue_full_system_proto_unix_io
    runtime_dir = args.blue_fullsystem_runtime_dir
    friendly_colour_yellow = False

    if args.run_yellow:
        proto_unix_io = tscope.yellow_full_system_proto_unix_io
        runtime_dir = args.yellow_fullsystem_runtime_dir
        friendly_colour_yellow = True

    if args.run_blue or args.run_yellow:
        # TODO (#2585): Support multiple channels
        with RobotCommunication(
            proto_unix_io, ROBOT_MULTICAST_CHANNEL_0, args.interface
        ), FullSystem(
            runtime_dir, args.debug_fullsystem, friendly_colour_yellow
        ) as full_system:
            tscope.load_saved_layout(
                args.layout, load_blue=args.run_blue, load_yellow=args.run_yellow
            )
            tscope.show()

    ###########################################################################
    #           Blue AI vs Yellow AI + Simulator + Gamecontroller             #
    ###########################################################################
    #
    # Run two AIs against each other with the er-force simulator. We also run
    # the gamecontroller which can be accessed from http://localhost:8081
    #
    # The async sim ticket ticks the simulator at a fixed rate.
    else:

        def __async_sim_ticker(tick_rate_ms):
            """Setup the world and tick simulation forever

            :param tick_rate_ms: The tick rate of the simulation

            """
            world_state = tbots_protobuf.create_world_state(
                [geom.Point(3, y) for y in numpy.linspace(-2, 2, NUM_ROBOTS)],
                [geom.Point(-3, y) for y in numpy.linspace(-2, 2, NUM_ROBOTS)],
                ball_location=geom.Point(0, 0),
                ball_velocity=geom.Vector(0, 0),
            )
            tscope.simulator_proto_unix_io.send_proto(WorldState, world_state)

            # Tick Simulation
            while True:
                tick = SimulatorTick(milliseconds=tick_rate_ms)
                tscope.simulator_proto_unix_io.send_proto(SimulatorTick, tick)
                time.sleep(tick_rate_ms / 1000)

        # Launch all binaries
        with Simulator(
            args.simulator_runtime_dir, args.debug_simulator
        ) as simulator, FullSystem(
            args.blue_fullsystem_runtime_dir, args.debug_fullsystem, False
        ) as blue_fs, FullSystem(
            args.yellow_fullsystem_runtime_dir, args.debug_fullsystem, True
        ) as yellow_fs, Gamecontroller() as gamecontroller:

            blue_fs.setup_proto_unix_io(tscope.blue_full_system_proto_unix_io)
            yellow_fs.setup_proto_unix_io(tscope.yellow_full_system_proto_unix_io)
            simulator.setup_proto_unix_io(
                tscope.simulator_proto_unix_io,
                tscope.blue_full_system_proto_unix_io,
                tscope.yellow_full_system_proto_unix_io,
            )
            gamecontroller.setup_proto_unix_io(
                tscope.blue_full_system_proto_unix_io,
                tscope.yellow_full_system_proto_unix_io,
            )

            tscope.load_saved_layout(args.layout)

            # Start the simulator
            thread = threading.Thread(
                target=__async_sim_ticker, args=(SIM_TICK_RATE_MS,), daemon=True,
            )

            thread.start()
            tscope.show()
            thread.join()
            tscope.close()
