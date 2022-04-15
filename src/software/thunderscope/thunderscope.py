import os
import atexit
import signal
import threading
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

import pyqtgraph
import qdarktheme
import software.python_bindings as geom

from subprocess import Popen
from pyqtgraph.dockarea import *
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *

from proto.import_all_protos import *
from proto.message_translation import tbots_protobuf
from extlibs.er_force_sim.src.protobuf.world_pb2 import (
    SimulatorState,
    SimBall,
    SimRobot,
)

from software.py_constants import *
from software.networking import threaded_unix_sender, networking
from software.thunderscope.arbitrary_plot.named_value_plotter import NamedValuePlotter
from software.estop.estop_reader import ThreadedEstopReader
from software.thunderscope.field import (
    obstacle_layer,
    path_layer,
    validation_layer,
    simulator_layer,
    world_layer,
)
from software.thunderscope.field.field import Field
from software.thunderscope.log.g3log_widget import g3logWidget
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.play.playinfo_widget import playInfoWidget
from software.thunderscope.robot_diagnostics.chicker import ChickerWidget
from software.thunderscope.robot_diagnostics.drive_and_dribbler_widget import (
    DriveAndDribblerWidget,
)

DEFAULT_LAYOUT_PATH = "/opt/tbotspython/default_tscope_layout"
PROCESS_LAUNCH_DELAY_S = 1
NUM_ROBOTS = 6

pyqtgraph.setConfigOption("antialias", True)


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

    Thunderscope also manages our main binaries (fullsystem, simulator,
    gamecontroller, etc..) and sets up the communication between them (and
    thunderscope) using ProtoUnixIO instances.

    """

    def __init__(
        self,
        simulator_runtime_dir,
        blue_fullsystem_runtime_dir,
        yellow_fullsystem_runtime_dir,
        debug_fullsystem=False,
        debug_simulator=False,
        refresh_interval_ms=5,
    ):
        """Initialize Thunderscope

        :param simulator_runtime_dir: The directory to run the simulator in
        :param blue_fullsystem_runtime_dir: The directory to run the blue fullsystem in
        :param yellow_fullsystem_runtime_dir: The directory to run the yellow fullsystem in
        :param refresh_interval_ms: The interval in milliseconds to refresh the simulator
        :param debug_fullsystem: Whether to run the fullsystem in debug mode
        :param debug_simulator: Whether to run the simulator in debug mode

        """

        self.simulator_runtime_dir = simulator_runtime_dir
        self.blue_fullsystem_runtime_dir = blue_fullsystem_runtime_dir
        self.yellow_fullsystem_runtime_dir = yellow_fullsystem_runtime_dir
        self.debug_fullsystem = debug_fullsystem
        self.debug_simulator = debug_simulator
        self.refresh_interval_ms = refresh_interval_ms

        # Setup MainApp and initialize DockArea
        self.app = pyqtgraph.mkQApp("Thunderscope")
        self.app.setStyleSheet(qdarktheme.load_stylesheet())

        signal.signal(signal.SIGINT, signal.SIG_DFL)

        self.main_dock = DockArea()
        self.blue_full_system_dock_area = DockArea()
        self.yellow_full_system_dock_area = DockArea()

        gamecontroller_dock = self.setup_gamecontroller_widget()
        self.main_dock.addDock(gamecontroller_dock, "below")

        blue_dock = Dock("Blue Fullsystem")
        blue_dock.addWidget(self.blue_full_system_dock_area)

        yellow_dock = Dock("Yellow Fullsystem")
        yellow_dock.addWidget(self.yellow_full_system_dock_area)

        self.main_dock.addDock(yellow_dock, "above", gamecontroller_dock)
        self.main_dock.addDock(blue_dock, "above", yellow_dock)

        self.window = QtGui.QMainWindow()
        self.window.setCentralWidget(self.main_dock)
        self.window.setWindowTitle("Thunderscope")
        self.last_refresh_time = time.time()

        # ProtoUnixIOs
        #
        # NOTE: We have two separate IOs for each full system because the
        # er force simulator expects two inputs of the same protobuf type but
        # from the blue or yellow team. We also would like to visualize the same
        # protobuf types on two separate widgets.
        self.simulator_proto_unix_io = ProtoUnixIO()
        self.yellow_full_system_proto_unix_io = ProtoUnixIO()
        self.blue_full_system_proto_unix_io = ProtoUnixIO()

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
            if os.path.exists(DEFAULT_LAYOUT_PATH):
                os.remove(DEFAULT_LAYOUT_PATH)
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
                """
                Cntrl+S: Save Layout
                Cntrl+O: Open Layout
                Double Click Purple Bar to pop window out
                Drag Purple Bar to rearrange docks
                Click items in legends to select/deselect

                Cntrl-Click and Drag: Move ball and kick
                Click and Drag Robots to move them around
                """,
            )
        )

    def __enter__(self):
        """Enter the Thunderscope context manager. 

        Launches the simulator and the blue and yellow fullsystems. Also starts the
        gamecontroller. We use the context manager to launch these binaries, so that
        we can cleanly kill them when we exit on any error, signal, etc..

        If the debug mode is enabled (debug_fullsystem or debug_simulator), then
        the binary is _not_ run and the command to debug under gdb is printed.

        :return: Thunderscope instance

        """
        self.running = True

        yellow_fullsystem_command = "software/unix_full_system --runtime_dir={} {}".format(
            self.yellow_fullsystem_runtime_dir, "--friendly_colour_yellow"
        )
        blue_fullsystem_command = "software/unix_full_system --runtime_dir={}".format(
            self.blue_fullsystem_runtime_dir
        )
        simulator_command = "software/er_force_simulator_main --runtime_dir={}".format(
            self.simulator_runtime_dir
        )

        if self.debug_fullsystem:
            print("============ Debugging Full System ==============")

            print("\n1. Build full system in debug mode")
            print("./tbots.py -d build unix_full_system")

            print("\n2. Run the following binaries from src to debug full system")
            print("gdb --args bazel-bin/{}".format(yellow_fullsystem_command))
            print("gdb --args bazel-bin/{}\n".format(blue_fullsystem_command))

        else:
            self.yellow_full_system_proc = Popen(yellow_fullsystem_command.split(" "))
            self.blue_full_system_proc = Popen(blue_fullsystem_command.split(" "))

        if self.debug_simulator:
            print("============== Debugging Simulator ==============")

            print("\n1. Build the simulator in debug mode:")
            print("./tbots.py -d build er_force_simulator_main")

            print("\n2. Run the following binary from src to debug the simulator:")
            print("gdb --args bazel-bin/{}\n".format(simulator_command))

        else:
            self.er_force_simulator_proc = Popen(simulator_command.split(" "))

        # Wait for user to start the gdb/debugging instances
        if self.debug_fullsystem or self.debug_simulator:
            input("After launching the gdb instances, press enter to continue...")

        self.gamecontroller_proc = Popen(["/opt/tbotspython/gamecontroller"])

        # Setup proto unix IO
        self.__setup_fullsystem_proto_unix_io(
            self.blue_fullsystem_runtime_dir, self.blue_full_system_proto_unix_io
        )
        self.__setup_fullsystem_proto_unix_io(
            self.yellow_fullsystem_runtime_dir, self.yellow_full_system_proto_unix_io
        )
        self.__setup_simulator_proto_unix_io(self.simulator_runtime_dir)
        self.__setup_gamecontroller_proto_unix_io()

        time.sleep(PROCESS_LAUNCH_DELAY_S)

        return self

    def __exit__(self, type, value, traceback):
        """Exit the Thunderscope context manager.

        Kill the fullsystems, simulator, and gamecontroller.

        :param type: The type of exception that was raised
        :param value: The exception that was raised
        :param traceback: The traceback of the exception

        """
        self.running = False

        for proc in [
            self.gamecontroller_proc,
            self.er_force_simulator_proc,
            self.yellow_full_system_proc,
            self.blue_full_system_proc,
        ]:
            if proc:
                proc.kill()
                proc.wait()

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
            shelf["dock_state"] = self.main_dock.saveState()
            shelf["blue_dock_state"] = self.blue_full_system_dock_area.saveState()
            shelf["yellow_dock_state"] = self.yellow_full_system_dock_area.saveState()

        with shelve.open(DEFAULT_LAYOUT_PATH, "c") as shelf:
            shelf["dock_state"] = self.main_dock.saveState()
            shelf["blue_dock_state"] = self.blue_full_system_dock_area.saveState()
            shelf["yellow_dock_state"] = self.yellow_full_system_dock_area.saveState()

    def load_layout(self, filename=None):
        """Open a file dialog to load the layout and state to all
        widgets

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
            self.main_dock.restoreState(shelf["dock_state"], missing="ignore")
            self.blue_full_system_dock_area.restoreState(
                shelf["blue_dock_state"], missing="ignore"
            )
            self.yellow_full_system_dock_area.restoreState(
                shelf["yellow_dock_state"], missing="ignore"
            )

            # Update default layout
            if filename != DEFAULT_LAYOUT_PATH:
                with shelve.open(DEFAULT_LAYOUT_PATH, "c") as default_shelf:
                    default_shelf["dock_state"] = shelf["dock_state"]
                    default_shelf["blue_dock_state"] = shelf["blue_dock_state"]
                    default_shelf["yellow_dock_state"] = shelf["yellow_dock_state"]
                    default_shelf.sync()

    def load_saved_layout(self, layout_path):
        """Load the specified layout or the default file. If the default layout
        file doesn't exist, and no layout is provided, then just configure
        the default layout.

        :param layout_path: Path to the layout file to load.

        """
        self.configure_default_layout(
            self.yellow_full_system_dock_area,
            self.simulator_proto_unix_io,
            self.yellow_full_system_proto_unix_io,
            True,
        )

        self.configure_default_layout(
            self.blue_full_system_dock_area,
            self.simulator_proto_unix_io,
            self.blue_full_system_proto_unix_io,
            False,
        )

        path = layout_path if layout_path else DEFAULT_LAYOUT_PATH

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

    def __setup_fullsystem_proto_unix_io(self, runtime_dir, proto_unix_io):
        """Helper to run full system and attach the appropriate unix senders/listeners

        :param runtime_dir: The runtime directory to run fullsystem
        :param proto_unix_io: The unix io to setup for this fullsystem instance

        """

        # Setup unix socket directory
        try:
            os.mkdir(runtime_dir)
        except:
            pass

        # Setup LOG(VISUALIZE) handling from full system. We set from_log_visualize
        # to true.
        for arg in [
            (runtime_dir, Obstacles, True),
            (runtime_dir, PathVisualization, True),
            (runtime_dir, NamedValue, True),
            (runtime_dir, PlayInfo, True),
        ]:
            proto_unix_io.attach_unix_receiver(*arg)

        proto_unix_io.attach_unix_receiver(runtime_dir + "/log", RobotLog)

        # Inputs to full_system
        for arg in [
            (runtime_dir + ROBOT_STATUS_PATH, RobotStatus),
            (runtime_dir + SSL_WRAPPER_PATH, SSL_WrapperPacket),
            (runtime_dir + SSL_REFEREE_PATH, Referee),
            (runtime_dir + SENSOR_PROTO_PATH, SensorProto),
            (runtime_dir + TACTIC_OVERRIDE_PATH, AssignedTacticPlayControlParams),
        ]:
            proto_unix_io.attach_unix_sender(*arg)

        # Outputs from full_system
        proto_unix_io.attach_unix_receiver(runtime_dir + WORLD_PATH, World)
        proto_unix_io.attach_unix_receiver(runtime_dir + PRIMITIVE_PATH, PrimitiveSet)

    def __setup_simulator_proto_unix_io(self, simulator_runtime_dir):
        """Setup the proto unix io for the simulator

        :param simulator_runtime_dir: The runtime directory of the simulator.

        """
        # Setup unix socket directory
        try:
            os.mkdir(simulator_runtime_dir)
        except:
            pass

        # inputs to er_force_simulator_main
        for arg in [
            (simulator_runtime_dir + SIMULATION_TICK_PATH, SimulatorTick),
            (simulator_runtime_dir + WORLD_STATE_PATH, WorldState),
            (simulator_runtime_dir + SIMULATOR_STATE_PATH, SimulatorState),
        ]:
            self.simulator_proto_unix_io.attach_unix_sender(*arg)

        # setup blue full system unix io
        for arg in [
            (simulator_runtime_dir + BLUE_WORLD_PATH, World),
            (simulator_runtime_dir + BLUE_PRIMITIVE_SET, PrimitiveSet),
        ]:
            self.blue_full_system_proto_unix_io.attach_unix_sender(*arg)

        for arg in [
            (simulator_runtime_dir + BLUE_SSL_WRAPPER_PATH, SSL_WrapperPacket),
            (simulator_runtime_dir + BLUE_ROBOT_STATUS_PATH, RobotStatus),
        ]:
            self.blue_full_system_proto_unix_io.attach_unix_receiver(*arg)

        # setup yellow full system unix io
        for arg in [
            (simulator_runtime_dir + YELLOW_WORLD_PATH, World),
            (simulator_runtime_dir + YELLOW_PRIMITIVE_SET, PrimitiveSet),
        ]:
            self.yellow_full_system_proto_unix_io.attach_unix_sender(*arg)

        for arg in [
            (simulator_runtime_dir + YELLOW_SSL_WRAPPER_PATH, SSL_WrapperPacket),
            (simulator_runtime_dir + YELLOW_ROBOT_STATUS_PATH, RobotStatus),
        ]:
            self.yellow_full_system_proto_unix_io.attach_unix_receiver(*arg)

    def __setup_gamecontroller_proto_unix_io(self):
        """Setup gamecontroller io
        """

        def __send_referee_command(data):
            self.blue_full_system_proto_unix_io.send_proto(Referee, data)
            self.yellow_full_system_proto_unix_io.send_proto(Referee, data)

        # TODO pull the constants out into a file
        self.receive_referee_command = networking.SSLRefereeProtoListener(
            "224.5.23.1", 10003, __send_referee_command, True,
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
        print("Configuring default layout", friendly_colour_yellow)
        # Configure Docks
        field_dock = self.setup_field_widget(
            sim_proto_unix_io, full_system_proto_unix_io, friendly_colour_yellow
        )
        log_dock = self.setup_log_widget(full_system_proto_unix_io)
        performance_dock = self.setup_performance_plot(full_system_proto_unix_io)
        play_info_dock = self.setup_play_info(full_system_proto_unix_io)

        dock_area.addDock(field_dock)
        dock_area.addDock(log_dock, "bottom", field_dock)
        dock_area.addDock(performance_dock, "right", log_dock)
        dock_area.addDock(play_info_dock, "right", performance_dock)

    def setup_field_widget(
        self, sim_proto_unix_io, full_system_proto_unix_io, friendly_colour_yellow
    ):
        """setup the field widget with the constituent layers

        :param sim_proto_unix_io: The proto unix io object for the simulator
        :param full_system_proto_unix_io: The proto unix io object for the full system
        :param friendly_colour_yellow: Whether the friendly colour is yellow
        :returns: the dock containing the field widget

        """
        self.field = Field()

        # Create layers
        paths = path_layer.PathLayer()
        obstacles = obstacle_layer.ObstacleLayer()
        validation = validation_layer.ValidationLayer()
        world = world_layer.WorldLayer(sim_proto_unix_io, friendly_colour_yellow)
        sim_state = simulator_layer.SimulatorLayer(friendly_colour_yellow)

        # Add field layers to field
        self.field.add_layer("Vision", world)
        self.field.add_layer("Obstacles", obstacles)
        self.field.add_layer("Paths", paths)
        self.field.add_layer("Validation", validation)
        self.field.add_layer("Simulator Layer", sim_state)

        # Register observers
        sim_proto_unix_io.register_observer(
            SimulatorState, sim_state.simulator_state_buffer
        )

        for arg in [
            (World, world.world_buffer),
            (Obstacles, obstacles.obstacle_buffer),
            (PathVisualization, paths.path_visualization_buffer),
            (ValidationProtoSet, validation.validation_set_buffer),
        ]:
            full_system_proto_unix_io.register_observer(*arg)

        # Register refresh functions
        self.register_refresh_function(self.field.refresh)

        # Create and return dock
        field_dock = Dock("Field")
        field_dock.addWidget(self.field)

        return field_dock

    def setup_gamecontroller_widget(self):
        """Setup the gamecontroller widget

        :param proto_unix_io: The proto unix io object
        :return the gamecontroller in a dock

        """
        web_view = QWebEngineView()
        web_view.load(QtCore.QUrl("http://localhost:8081"))

        # create and return dock
        gamecontroller_dock = Dock("GameController")
        gamecontroller_dock.addWidget(web_view)

        return gamecontroller_dock

    def setup_log_widget(self, proto_unix_io):
        """Setup the wiget that receives logs from full system

        :param proto_unix_io: The proto unix io object
        :returns: The dock containing the log widget

        """
        # Create widget
        self.logs = g3logWidget()

        # Register observer
        proto_unix_io.register_observer(RobotLog, self.logs.log_buffer)

        # Register refresh function
        self.register_refresh_function(self.logs.refresh)

        # Create and return dock
        log_dock = Dock("Logs")
        log_dock.addWidget(self.logs)

        return log_dock

    def setup_performance_plot(self, proto_unix_io):
        """Setup the performance plot

        :param proto_unix_io: The proto unix io object
        :returns: The performance plot setup in a dock

        """
        # Create widget
        self.named_value_plotter = NamedValuePlotter()

        # Register observer
        proto_unix_io.register_observer(
            NamedValue, self.named_value_plotter.named_value_buffer
        )

        # Register refresh function
        self.register_refresh_function(self.named_value_plotter.refresh)

        # Create and return dock
        named_value_plotter_dock = Dock("Performance")
        named_value_plotter_dock.addWidget(self.named_value_plotter.win)
        return named_value_plotter_dock

    def setup_play_info(self, proto_unix_io):
        """Setup the play info widget

        :param proto_unix_io: The proto unix io object
        :returns: The play info widget setup in a dock

        """

        play_info = playInfoWidget()
        proto_unix_io.register_observer(PlayInfo, play_info.playinfo_buffer)
        self.register_refresh_function(play_info.refresh)

        play_info_dock = Dock("playInfo")
        play_info_dock.addWidget(play_info)

        return play_info_dock

    def setup_chicker_widget(self, proto_unix_io):
        """Setup the chicker widget for robot diagnostics

        :param proto_unix_io: The proto unix io object
        :returns: The dock containing the chicker widget

        """
        # Create widget
        self.chicker_widget = ChickerWidget()

        # Register refresh function
        self.register_refresh_function(self.chicker_widget.refresh)

        chicker_dock = Dock("Chicker")
        chicker_dock.addWidget(self.chicker_widget)

        return chicker_dock

    def setup_drive_and_dribbler_widget(self):
        """Setup the drive and dribbler widget"""

        drive_and_dribbler = DriveAndDribblerWidget()

        drive_and_dribbler_dock = Dock("Drive and Dribbler")
        drive_and_dribbler_dock.addWidget(drive_and_dribbler)

        return drive_and_dribbler_dock

    def show(self):
        """Show the main window"""

        self.window.show()
        pyqtgraph.exec()

    def close(self):
        QtCore.QTimer.singleShot(0, self.window.close)


if __name__ == "__main__":

    # Setup parser
    parser = argparse.ArgumentParser(description="Thunderscope")
    parser.add_argument(
        "--robot_diagnostics",
        action="store_true",
        help="Run thunderscope in the robot diagnostics configuration",
    )
    parser.add_argument(
        "--layout",
        action="store",
        help="Which layout to run, if not specified the last layout will run",
    )
    parser.add_argument(
        "--sim_tick_rate_ms",
        action="store",
        type=int,
        default=16,
        help="The tick rate of the simulator in ms",
    )
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
    args = parser.parse_args()

    # Robot Diagnostics
    if args.robot_diagnostics:

        estop_reader = ThreadedEstopReader("/dev/ttyACM0", 115200)

        thunderscope = Thunderscope()
        log_dock = thunderscope.setup_log_widget()
        thunderscope.dock_area.addDock(log_dock)

        chicker_dock = thunderscope.setup_chicker_widget()
        thunderscope.dock_area.addDock(chicker_dock)

        drive_and_dribbler_dock = thunderscope.setup_drive_and_dribbler_widget()
        thunderscope.dock_area.addDock(drive_and_dribbler_dock)
        thunderscope.show()

    # Default (AI vs AI + Simulator)
    #
    # Asynchronously tick the simulator to run both full systems at the
    # provided rate.
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
            while tscope.running:
                tick = SimulatorTick(milliseconds=tick_rate_ms)
                tscope.simulator_proto_unix_io.send_proto(SimulatorTick, tick)
                time.sleep(tick_rate_ms / 1000)

        with Thunderscope(
            args.simulator_runtime_dir,
            args.blue_fullsystem_runtime_dir,
            args.yellow_fullsystem_runtime_dir,
            args.debug_fullsystem,
            args.debug_simulator,
        ) as tscope:

            tscope.load_saved_layout(args.layout)

            thread = threading.Thread(
                target=__async_sim_ticker, args=(args.sim_tick_rate_ms,), daemon=True,
            )

            thread.start()
            tscope.show()
            thread.join()
