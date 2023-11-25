import time
import shelve
import logging
import pathlib
import os

import pyqtgraph
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *

from typing import Callable

from software.py_constants import *
from software.thunderscope.constants import *

from software.thunderscope.thunderscope_config import TScopeConfig


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
        layout_path: os.PathLike = None,
        refresh_interval_ms: int = 10,
    ) -> None:
        """Initialize Thunderscope

        :param config: The current Thunderscope UI configuration
        :param layout_path: The path to the layout to load
        :param refresh_interval_ms:
            The interval in milliseconds to refresh all the widgets.

        """

        self.refresh_interval_ms = refresh_interval_ms
        self.widgets = {}
        self.refresh_timers = []

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
        self.proto_unix_io_map = config.proto_unix_io_map
        self.tab_dock_map = {}

        # iterate through each tab and add one by one
        for tab in config.tabs:
            self.tab_dock_map[tab.name] = tab.dock_area
            self.tabs.addTab(tab.dock_area, tab.name)
            self.register_refresh_function(tab.refresh)

        self.window = QMainWindow()
        self.window.setCentralWidget(self.tabs)
        self.window.setWindowIcon(
            QtGui.QIcon("software/thunderscope/thunderscope-logo.png")
        )
        self.window.setWindowTitle("Thunderscope")

        # Load the layout file if it exists
        path = layout_path if layout_path else LAST_OPENED_LAYOUT_PATH
        try:
            self.load_layout(path)
        except Exception:
            pass

        # Keyboard Estop shortcut
        # only used when in keyboard estop mode
        self.keyboard_estop_shortcut = QtGui.QShortcut(
            QtGui.QKeySequence(" "), self.window
        )

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
            lambda: QMessageBox.information(self.window, "Help", THUNDERSCOPE_HELP_TEXT)
        )

    def reset_layout(self) -> None:
        """Reset the layout to the default layout"""
        saved_layout_path = pathlib.Path(LAST_OPENED_LAYOUT_PATH)
        saved_layout_path.unlink(missing_ok=True)
        QMessageBox.information(
            self.window,
            "Restart Required",
            "Restart thunderscope to reset the layout.",
        )

    def save_layout(self) -> None:
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

        filename, _ = QFileDialog.getSaveFileName(
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

    def load_layout(self, filename: os.PathLike = None) -> None:
        """Open a file dialog to load the layout and state to all widgets

        :param filename: The filename to load the layout from. If None, then
                         open a file dialog.

        """

        if filename is None:
            filename, _ = QFileDialog.getOpenFileName(
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
                if key in self.tab_dock_map:
                    self.tab_dock_map[key].restoreState(val)

            # Update default layout
            if filename != LAST_OPENED_LAYOUT_PATH:
                with shelve.open(LAST_OPENED_LAYOUT_PATH, "c") as default_shelf:
                    for key, val in shelf.items():
                        default_shelf[key] = val
                    default_shelf.sync()

    def register_refresh_function(self, refresh_func: Callable[[], None]) -> None:
        """Register the refresh functions to run at the refresh_interval_ms
        passed into thunderscope.

        :param refresh_func: The function to call at refresh_interval_ms

        """
        refresh_timer = QtCore.QTimer()
        refresh_timer.setTimerType(QtCore.Qt.TimerType.PreciseTimer)
        refresh_timer.timeout.connect(lambda: refresh_func())
        refresh_timer.start(self.refresh_interval_ms)

        self.refresh_timers.append(refresh_timer)

    def show(self) -> None:
        """Show the main window"""

        self.window.show()
        self.window.showMaximized()
        pyqtgraph.exec()

    def is_open(self) -> bool:
        """Returns true if the window is open"""
        return self.window.isVisible()

    def close(self) -> None:
        """Close the main window"""

        QtCore.QTimer.singleShot(0, self.window.close)
