import time
import textwrap
import shelve
import signal
import logging
import pathlib

import PyQt6
from PyQt6.QtWebEngineWidgets import QWebEngineView

from qt_material import apply_stylesheet, list_themes

import pyqtgraph
import qdarktheme
from pyqtgraph.dockarea import *
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *

from typing import cast

from software.py_constants import *
from proto.import_all_protos import *
from extlibs.er_force_sim.src.protobuf.world_pb2 import *
from software.thunderscope.dock_label_style import *

from software.thunderscope.thunderscope_config_types import *
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.constants import TabKeys

SAVED_LAYOUT_PATH = "/opt/tbotspython/saved_tscope_layout"
LAYOUT_FILE_EXTENSION = "tscopelayout"
LAST_OPENED_LAYOUT_PATH = (
    f"{SAVED_LAYOUT_PATH}/last_opened_tscope_layout.{LAYOUT_FILE_EXTENSION}"
)


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
        config: TScopeConfig,
        simulator_proto_unix_io=None,
        blue_full_system_proto_unix_io=None,
        yellow_full_system_proto_unix_io=None,
        layout_path=None,
        refresh_interval_ms=10,
    ):
        """Initialize Thunderscope

        :param config: The current Thunderscope UI configuration
        :param simulator_proto_unix_io: The simulator's proto unix io
        :param blue_full_system_proto_unix_io: The blue full system's proto unix io
        :param yellow_full_system_proto_unix_io: The yellow full system's proto unix io
        :param layout_path: The path to the layout to load
        :param refresh_interval_ms:
            The interval in milliseconds to refresh all the widgets.

        """
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        # Setup MainApp and initialize DockArea
        self.app = pyqtgraph.mkQApp("Thunderscope")

        # Setup stylesheet
        apply_stylesheet(self.app, theme="dark_blue.xml")

        self.refresh_interval_ms = refresh_interval_ms
        self.widgets = {}
        self.refresh_timers = []

        # TODO (#2586) Improve this layout
        self.tabs = QTabWidget()

        # ProtoUnixIOs
        #
        # NOTE: Simulated tests need to be able to run without Thunderscope
        # enabled, so the test fixture creates its own ProtoUnixIOs. But, we
        # would optionally like to enable thunderscope, observe protos and plot
        # them in thunderscope. So we need to switch over to the dependency
        # injected ProtoUnixIOs when provided.

        # Also NOTE: We have two separate IOs for each full system because the
        # er force simulator expects two inputs of the same protobuf type but
        # from the blue or yellow team. We also would like to visualize the same
        # protobuf types on two separate widgets.

        # initialize proto unix io map with initial values or default if not provided
        self.proto_unix_io_map = {
            ProtoUnixIOTypes.BLUE: blue_full_system_proto_unix_io or ProtoUnixIO(),
            ProtoUnixIOTypes.YELLOW: yellow_full_system_proto_unix_io or ProtoUnixIO(),
            ProtoUnixIOTypes.SIM: simulator_proto_unix_io or ProtoUnixIO(),
        }
        self.widgets_map = {}
        self.tab_dock_map = {}
        self.widget_dock_map = {}

        # iterate through and initialise each non-ref proto unix io
        for name in list(config.protos.keys()):

            # if not a ref proto unix io
            if config.protos[name] is None:

                # if it hasn't already been passed in above, initialise it to default
                if name not in self.proto_unix_io_map:
                    self.proto_unix_io_map[name] = ProtoUnixIO()

                # remove once it's initialised
                del config.protos[name]

        # iterate through and initialise each ref proto unix io
        for name, ref in config.protos.items():

            # if ref exists, maps name to that ref's value
            # else, throws error
            if ref in self.proto_unix_io_map:
                self.proto_unix_io_map[name] = self.proto_unix_io_map[ref]
            else:
                raise NameError(f"Given Proto Unix IO Reference {ref} was not found")

        # iterate through each tab and add one by one
        for tab in config.tabs:

            # QT Tab, contains widgets
            if type(tab) == TScopeQTTab:
                tab = cast(TScopeQTTab, tab)
                # make dock area
                dock_area = DockArea()
                self.tabs.addTab(dock_area, tab.name)
                self.tab_dock_map[str(tab.key.name)] = dock_area

                self.widgets_map[tab.key] = {}
                self.widget_dock_map[tab.key] = {}

                # first widget is initial anchor widget
                # all other widgets will be positioned relative to this one
                for widget in tab.widgets:
                    self.add_one_widget(
                        dock_area,
                        widget,
                        self.widgets_map[tab.key],
                        self.widget_dock_map[tab.key],
                    )

            # WEB Tab, displays a Chrome webpage
            elif type(tab) == TScopeWebTab:
                tab = cast(TScopeWebTab, tab)
                self.web_view = QWebEngineView()
                self.web_view.load(QtCore.QUrl(tab.url))
                self.tabs.addTab(self.web_view, tab.name)

        self.window = QtGui.QMainWindow()
        self.window.setCentralWidget(self.tabs)
        self.window.setWindowIcon(
            QtGui.QIcon("software/thunderscope/thunderscope-logo.png")
        )
        self.window.setWindowTitle("Thunderscope")

        # save robot view control mode signal as a field
        for tab_widgets in self.widgets_map.values():
            if "Robot View" in tab_widgets:
                self.control_mode_signal = tab_widgets["Robot View"].control_mode_signal

        # Load the layout file if it exists
        path = layout_path if layout_path else LAST_OPENED_LAYOUT_PATH
        try:
            self.load_layout(path)
        except Exception:
            pass

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
                    
                    Double Click Blue Bar to pop window out
                    Drag Blue Bar to rearrange docks
                    Click items in legends to select/deselect
                    Cntrl-Click and Drag: Move ball and kick
                    Cntrl-Space: Stop AI vs AI simulation
                    """
                ),
            )
        )

    def get_widget_args(self, deps):
        """
        Takes in a list of dependencies and converts it into a dict of arg names to values
        So that it can be unpacked and passed into a function

        :param deps: list of dependencies
        :return: dict of function argument names to values
        """
        args = {}

        for dep in deps:
            if dep.type == ParamTypes.PROTO_UNIX_IO:
                args[dep.name] = self.proto_unix_io_map[dep.value]
            else:
                args[dep.name] = dep.value

        return args

    def reset_layout(self):
        """Reset the layout to the default layout"""
        saved_layout_path = pathlib.Path(LAST_OPENED_LAYOUT_PATH)
        saved_layout_path.unlink(missing_ok=True)
        QMessageBox.information(
            self.window,
            "Restart Required",
            "Restart thunderscope to reset the layout.",
        )

    def save_layout(self):
        """Open a file dialog to save the layout and any other
        registered state to a file

        """
        # Create a folder at SAVED_LAYOUT_PATH if it doesn't exist
        try:
            pathlib.Path(SAVED_LAYOUT_PATH).mkdir(exist_ok=True)
        except FileNotFoundError:
            logging.warning(
                f"Could not create folder at '{SAVED_LAYOUT_PATH}' for layout files"
            )

        filename, _ = QtGui.QFileDialog.getSaveFileName(
            self.window,
            "Save layout",
            f"{SAVED_LAYOUT_PATH}/dock_layout_{int(time.time())}.{LAYOUT_FILE_EXTENSION}",
            options=QFileDialog.Option.DontUseNativeDialog,
        )

        if not filename:
            # No layout file was selected
            return

        with shelve.open(filename, "c") as shelf:
            for key, val in self.tab_dock_map.items():
                shelf[key] = val.saveState()

        with shelve.open(LAST_OPENED_LAYOUT_PATH, "c") as shelf:
            for key, val in self.tab_dock_map.items():
                shelf[key] = val.saveState()

    def load_layout(self, filename=None):
        """Open a file dialog to load the layout and state to all widgets

        :param filename: The filename to load the layout from. If None, then
                         open a file dialog.

        """

        if filename is None:
            filename, _ = QtGui.QFileDialog.getOpenFileName(
                self.window,
                "Open layout",
                f"{SAVED_LAYOUT_PATH}/",
                options=QFileDialog.Option.DontUseNativeDialog,
            )

            if not filename:
                logging.warning("No filename selected")
                return

        # lets load the layouts from the shelf into their respective dock areas
        # if the dock doesn't exist in the default layout, we ignore it
        # (instead of adding a placeholder dock)
        with shelve.open(filename, "r") as shelf:
            for key, val in shelf.items():
                if key in self.widget_dock_map:
                    self.tab_dock_map[key].restoreState(
                        val
                    )

            # Update default layout
            if filename != LAST_OPENED_LAYOUT_PATH:
                with shelve.open(LAST_OPENED_LAYOUT_PATH, "c") as default_shelf:
                    for key, val in shelf.items():
                        default_shelf[key] = val
                    default_shelf.sync()

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

    def add_one_widget(self, dock_area, data: TScopeWidget, widgets_map, dock_map):
        """
        Constructs a widget and dock with the given data and adds it to the given dock area
        As well as to the given map of widgets and docks
        :param dock_area: the dock area to add the widget to
        :param data: the data describing the widget of type TScopeWidget
        :param widgets_map: the map of all widgets to add to
        :param dock_map: the map of all docks to add to
        """
        widget_args = self.get_widget_args(data.deps)

        widget_name = data.name
        new_widget = data.setup(**widget_args)

        widgets_map[widget_name] = new_widget
        new_dock = Dock(widget_name)
        new_dock.addWidget(new_widget.win if data.in_window else new_widget)
        dock_map[widget_name] = new_dock

        if data.stretch:
            stretch_data = data.stretch
            if stretch_data.y:
                new_dock.setStretch(y=stretch_data.y)
            if stretch_data.x:
                new_dock.setStretch(x=stretch_data.x)

        if data.anchor and data.position:
            dock_area.addDock(new_dock, data.position, dock_map[data.anchor])
        else:
            dock_area.addDock(new_dock)

        if not data.no_refresh:
            self.register_refresh_function(new_widget.refresh)

    def show(self):
        """Show the main window"""

        self.window.show()
        self.window.showMaximized()
        pyqtgraph.exec()

    def close(self):
        """Close the main window"""

        QtCore.QTimer.singleShot(0, self.window.close)
