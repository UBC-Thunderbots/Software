import os
import signal
from threading import Thread
import queue
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
import qdarktheme
from subprocess import Popen
from pyqtgraph.dockarea import *
from pyqtgraph.Qt import QtCore, QtGui
import software.python_bindings as geom
from pyqtgraph.Qt.QtWidgets import *
from PyQt6.QtWebEngineWidgets import QWebEngineView

from proto.import_all_protos import *
from software.py_constants import *

from software.networking import threaded_unix_sender
from software.networking import networking
from software.thunderscope.arbitrary_plot.named_value_plotter import NamedValuePlotter
from software.estop.estop_reader import ThreadedEstopReader
from software.thunderscope.field import (
    obstacle_layer,
    path_layer,
    validation_layer,
    world_layer,
)
from software.thunderscope.field.field import Field
from software.thunderscope.log.g3log_widget import g3logWidget
from software.thunderscope.proto_unix_io import ProtoUnixIO
#from software.thunderscope.robot_communication import mobile_gamepad
from software.thunderscope.proto_receiver import ProtoReceiver
from software.thunderscope.play.playinfo_widget import playInfoWidget
from software.thunderscope.chicker.chicker import ChickerWidget


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

    def __init__(self, proto_unix_io, refresh_interval_ms=2):

        # Setup MainApp and initialize DockArea
        self.app = pyqtgraph.mkQApp("Thunderscope")
        self.app.setStyleSheet(qdarktheme.load_stylesheet())

        signal.signal(signal.SIGINT, signal.SIG_DFL)

        self.main_dock_area = DockArea()
        self.blue_full_system_dock_area = DockArea()
        self.yellow_full_system_dock_area = DockArea()

        self.window = QtGui.QMainWindow()
        self.window.setCentralWidget(self.blue_full_system_dock_area)
        # self.main_dock_area.addDock(self.blue_full_system_dock_area)
        # self.main_dock_area.addDock(self.yellow_full_system_dock_area)

        self.window.setWindowTitle("Thunderscope")

        # Setup unix socket directory
        try:
            os.mkdir("/tmp/tbots")
        except:
            pass

        self.proto_unix_io = proto_unix_io
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

            result = self.blue_full_system_dock_area.saveState()

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
                self.blue_full_system_dock_area.restoreState(shelf["dock_state"])

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

    def run_full_system(self, runtime_dir, proto_unix_io, friendly_colour_yellow=False):
        """Run full system and attach the appropriate unix senders/listeners

        :param runtime_dir: The runtime directory to run fullsystem
        :param proto_unix_io: The unix io to setup
        :returns: Running full system process

        """
        # Setup unix socket directory
        try:
            os.mkdir(runtime_dir)
        except:
            pass

        proto_unix_io.attach_unix_receiver(
            runtime_dir + "/" + Obstacles.DESCRIPTOR.full_name,
            Obstacles,
            from_log_visualize=True,
        )
        proto_unix_io.attach_unix_receiver(
            runtime_dir + "/" + PathVisualization.DESCRIPTOR.full_name,
            PathVisualization,
            from_log_visualize=True,
        )
        proto_unix_io.attach_unix_receiver(
            runtime_dir + "/" + NamedValue.DESCRIPTOR.full_name,
            NamedValue,
            from_log_visualize=True,
        )
        proto_unix_io.attach_unix_receiver(runtime_dir + "/log", RobotLog)

        # inputs to full_system
        proto_unix_io.attach_unix_sender(runtime_dir + ROBOT_STATUS_PATH, RobotStatus)
        proto_unix_io.attach_unix_sender(
            runtime_dir + SSL_WRAPPER_PATH, SSL_WrapperPacket
        )
        proto_unix_io.attach_unix_sender(runtime_dir + SSL_REFEREE_PATH, Referee)
        proto_unix_io.attach_unix_sender(runtime_dir + SENSOR_PROTO_PATH, SensorProto)
        proto_unix_io.attach_unix_receiver(
            runtime_dir + TACTIC_OVERRIDE_PATH, AssignedTacticPlayControlParams
        )

        # outputs from full_system
        proto_unix_io.attach_unix_receiver(runtime_dir + WORLD_PATH, World)
        proto_unix_io.attach_unix_receiver(runtime_dir + PRIMITIVE_PATH, PrimitiveSet)

        # TODO (#2510) rename to full_system
        return Popen(
            "software/unix_full_system --runtime_dir={} {}".format(
                runtime_dir,
                "--friendly_colour_yellow" if friendly_colour_yellow else "",
            ).split(" ")
        )

    def run_er_force_simulator(
        self,
        simulator_runtime_dir,
        blue_runtime_dir,
        yellow_runtime_dir,
        simulator_io,
        blue_full_system_proto_unix_io,
        yellow_full_system_proto_unix_io,
    ):
        """Run er force simulator and set up the proto unix IO

        :param thunderscope_proto_unix_io: These protobufs will be managed
            by thunderscope. WorldState (setup the simulator) and
            SimulatorTick (step the simulation) 
        :param blue_full_system_proto_unix_io: The unix io for the blue full system 
        :param yellow_proto_unix_io: The unix io for the yellow full system

        NOTE: We have two separate IOs for each full system because the
        er force simulator expects two inputs of the same type but from the blue
        or yellow team.

        """
        try:
            os.mkdir(simulator_runtime_dir)
        except:
            pass

        print(simulator_runtime_dir, blue_runtime_dir, yellow_runtime_dir)

        # inputs to er_force_simulator_main
        simulator_io.attach_unix_sender(
            simulator_runtime_dir + SIMULATION_TICK_PATH, SimulatorTick
        )
        simulator_io.attach_unix_sender(
            simulator_runtime_dir + WORLD_STATE_PATH, WorldState
        )

        # setup blue full system unix io
        blue_full_system_proto_unix_io.attach_unix_sender(
            simulator_runtime_dir + BLUE_WORLD_PATH, World
        )
        blue_full_system_proto_unix_io.attach_unix_sender(
            simulator_runtime_dir + BLUE_PRIMITIVE_SET, PrimitiveSet
        )
        blue_full_system_proto_unix_io.attach_unix_receiver(
            simulator_runtime_dir + BLUE_SSL_WRAPPER_PATH, SSL_WrapperPacket
        )
        blue_full_system_proto_unix_io.attach_unix_receiver(
            simulator_runtime_dir + BLUE_ROBOT_STATUS_PATH, RobotStatus
        )

        # setup yellow full system unix io
        yellow_full_system_proto_unix_io.attach_unix_sender(
            simulator_runtime_dir + YELLOW_WORLD_PATH, World
        )
        yellow_full_system_proto_unix_io.attach_unix_sender(
            simulator_runtime_dir + YELLOW_PRIMITIVE_SET, PrimitiveSet
        )
        yellow_full_system_proto_unix_io.attach_unix_receiver(
            simulator_runtime_dir + YELLOW_SSL_WRAPPER_PATH, SSL_WrapperPacket
        )
        yellow_full_system_proto_unix_io.attach_unix_receiver(
            simulator_runtime_dir + YELLOW_ROBOT_STATUS_PATH, RobotStatus
        )

        self.simulator_process = Popen(["software/er_force_simulator_main"])

    def run_gamecontroller(self, blue_io, yellow_io):
        """Run the gamecontroller
        """
        self.gamecontroller = Popen(["/opt/tbotspython/gamecontroller"])

        def send_referee_command(data):
            blue_io.send_proto(Referee, data)
            yellow_io.send_proto(Referee, data)

        self.receive_referee_command = networking.SSLRefereeProtoListener(
            "224.5.23.1", 10003, send_referee_command, True,
        )

    def register_refresh_function(self, refresh_func):
        """Register the refresh functions to run at the refresh_interval_ms
        passed into thunderscope.

        :param refresh_func: The function to call at refresh_interval_ms

        """
        self.refresh_functions.append(refresh_func)

    def configure_default_layout(self, dock_area):
        """Configure the default layout for thunderscope
        """
        # Configure Docks
        field_dock = self.setup_field_widget()
        log_dock = self.setup_log_widget()
        performance_dock = self.setup_performance_plot()
<<<<<<< HEAD
        gamecontroller_dock = self.setup_gamecontroller_widget()

        dock_area.addDock(gamecontroller_dock, "left")
        dock_area.addDock(field_dock, "below", gamecontroller_dock)
        dock_area.addDock(log_dock, "bottom", field_dock)
        dock_area.addDock(performance_dock, "right", log_dock)
=======
        play_info_dock = self.setup_play_info()

        self.dock_area.addDock(field_dock, "left")
        self.dock_area.addDock(log_dock, "bottom", field_dock)
        self.dock_area.addDock(performance_dock, "right", log_dock)
        self.dock_area.addDock(play_info_dock, "right", performance_dock)
>>>>>>> 38ea7a3ec8f84b45a1bdaa18716668ab76bd17a6

    def setup_field_widget(self):
        """setup the field widget with the constituent layers

        :returns: the dock containing the field widget

        """
        self.field = Field()

        self.simulator_io = ProtoUnixIO()

        # Create layers
        world = world_layer.WorldLayer(self.simulator_io)
        obstacles = obstacle_layer.ObstacleLayer()
        paths = path_layer.PathLayer()
        validation = validation_layer.ValidationLayer()
        # sim_state_layer = simulator_layer.SimulatorLayer(self.simulator_io)

        # Add field layers to field
        self.field.add_layer("Vision", world)
        self.field.add_layer("Obstacles", obstacles)
        self.field.add_layer("Paths", paths)
        self.field.add_layer("Validation", validation)
        # self.field.add_layer("Simulator Layer", sim_state_layer)

        # Register observers
        buf = queue.Queue()
        self.proto_unix_io.register_observer(World, world.world_buffer)
        self.proto_unix_io.register_observer(PrimitiveSet, buf)
        self.proto_unix_io.register_observer(Obstacles, obstacles.obstacle_buffer)
        self.proto_unix_io.register_observer(
            PathVisualization, paths.path_visualization_buffer
        )

        # Register refresh functions
        self.register_refresh_function(self.field.refresh)

        # Create and return dock
        field_dock = Dock("Field", size=(500, 2000))
        field_dock.addWidget(self.field)

        return field_dock

    def setup_gamecontroller_widget(self):
        """setup the gamecontroller widget

        """
        web_view = QWebEngineView()
        web_view.load(QtCore.QUrl("http://localhost:8081"))

        # create and return dock
        gamecontroller_dock = Dock("GameController", size=(500, 2000))
        gamecontroller_dock.addWidget(web_view)

        return gamecontroller_dock

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
        self.proto_unix_io.register_observer(RobotLog, self.logs.log_buffer)

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
        self.proto_unix_io.register_observer(
            NamedValue, self.named_value_plotter.named_value_buffer
        )

        # Register refresh function
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

    def setup_chicker_widget(self):
        """Setup the chicker widget for robot diagnostics

        :returns: The dock containing the chicker widget
        """
        # Create widget
        self.chicker_widget = ChickerWidget()

        # Register refresh function
        self.register_refresh_function(self.chicker_widget.refresh)

        # Create and return dock
        chicker_dock = Dock("Chicker", size=(100, 100))
        chicker_dock.addWidget(self.chicker_widget)
        return chicker_dock

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

    def __setup_robots(robot_locations, team_colour):
        """Initializes the world from a list of robot locations

        :param robot_locations: A list of robot locations (index is robot id)
        :param team_colour: The color (either "blue" or "yellow")

        """
        world_state = WorldState()

        for x, robot_map in enumerate(
            [world_state.blue_robots, world_state.yellow_robots]
        ):

            for robot_id, robot_location in enumerate(robot_locations):
                robot_map[robot_id].CopyFrom(
                    RobotState(
                        global_position=Point(
                            x_meters=-3 if x == 0 else 3, y_meters=robot_location.y()
                        ),
                        global_orientation=Angle(radians=0),
                        global_velocity=Vector(
                            x_component_meters=0, y_component_meters=0
                        ),
                        global_angular_velocity=AngularVelocity(radians_per_second=0),
                    )
                )

        return world_state

    if args.robot_diagnostics:

        # estop_reader = ThreadedEstopReader("/dev/ttyACM0", 115200)

        thunderscope = Thunderscope()
        log_dock = thunderscope.setup_log_widget()
        thunderscope.dock_area.addDock(log_dock)

        chicker_dock = thunderscope.setup_chicker_widget()
        thunderscope.dock_area.addDock(chicker_dock)

        thunderscope.show()

    elif args.run_simulator:
        print(
            "TODO #2050, this isn't implemented, just run the current standalone simulator"
        )

    else:

        yellow_io = ProtoUnixIO()
        blue_io = ProtoUnixIO()

        thunderscope = Thunderscope(blue_io)
        thunderscope.configure_default_layout(thunderscope.blue_full_system_dock_area)

        thunderscope.run_full_system("/tmp/tbots/blue", blue_io, False)
        thunderscope.run_full_system("/tmp/tbots/yellow", yellow_io, True)
        thunderscope.run_er_force_simulator(
            "/tmp/tbots",
            "/tmp/tbots/blue",
            "/tmp/tbots/yellow",
            thunderscope.simulator_io,
            blue_io,
            yellow_io,
        )
        thunderscope.run_gamecontroller(blue_io, yellow_io)

        def ticker():
            import time

            time.sleep(1)
            world_state = __setup_robots(
                [geom.Point(-3, x) for x in range(-2, 3)], "blue"
            )
            thunderscope.simulator_io.send_proto(WorldState, world_state)
            while True:
                tick = SimulatorTick()
                tick.milliseconds = 16
                thunderscope.simulator_io.send_proto(SimulatorTick, tick)
                time.sleep(0.016)

        thread = Thread(target=ticker)
        thread.start()
        thunderscope.show()
