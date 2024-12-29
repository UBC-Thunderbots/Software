import textwrap
from typing import Callable, List
from PyQt6.QtGui import QKeySequence
from numpy import who
from pyqtgraph import QtCore
from pyqtgraph.Qt import QtGui
from pyqtgraph.Qt.QtWidgets import *
from proto.import_all_protos import *
from PyQt6.QtCore import Qt
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.constants import (
    CameraView,
    THUNDERSCOPE_HELP_TEXT,
    SIMULATION_SPEEDS,
)
import software.thunderscope.gl.widgets.toolbar_icons.sandbox_mode.icon_loader as icons
from software.thunderscope.common.common_widgets import ToggleableButton
from software.thunderscope.gl.widgets.gl_toolbar import GLToolbar


class GLFieldToolbar(GLToolbar):
    """Toolbar for the GL Field Widget

    Has buttons for measure mode, changing camera views, showing help info
    And for undoing / redoing robot state changes
    """

    def __init__(
        self,
        parent: QWidget,
        on_camera_view_change: Callable[[CameraView], None],
        on_measure_mode: Callable[[], None],
        layers_menu: QMenu,
        toolbars_menu: QMenu,
        sandbox_mode: bool = False,
    ):
        """Set up the toolbar with these buttons:

        - Layers select menu
        - Undo
        - Pause
        - Redo
        - Help
        - Measure Mode Toggle
        - Camera View Select menu

        :param parent: the parent to overlay this toolbar over
        :param on_camera_view_change: the callback function for when the camera view is changed
        :param on_measure_mode: the callback function for when measure mode is toggled
        :param layers_menu: the QMenu for the layers menu selection
        :param toolbars_menu: the QMenu for the toolbars menu selection
        :param sandbox_mode: if sandbox mode should be enabled
        """
        super(GLFieldToolbar, self).__init__(parent=parent)

        # Setup Layers button for toggling visibility of layers
        self.layers_button = QPushButton("layers")
        self.layers_button.setStyleSheet(self.get_button_style())
        self.layers_button.setMenu(layers_menu)

        # Set up View button for setting the camera position to standard views
        self.camera_view_button = QPushButton()
        self.camera_view_button.setToolTip("View")
        self.camera_view_button.setIcon(icons.get_view_icon(self.BUTTON_ICON_COLOR))
        self.camera_view_button.setStyleSheet(self.get_button_style())
        self.camera_view_menu = QMenu()
        self.camera_view_button.setMenu(self.camera_view_menu)
        self.camera_view_actions = [
            QtGui.QAction("[1] Orthographic Top Down"),
            QtGui.QAction("[2] Landscape High Angle"),
            QtGui.QAction("[3] Left Half High Angle"),
            QtGui.QAction("[4] Right Half High Angle"),
        ]
        self.camera_view_actions[0].triggered.connect(
            lambda: on_camera_view_change(CameraView.ORTHOGRAPHIC)
        )
        self.camera_view_actions[1].triggered.connect(
            lambda: on_camera_view_change(CameraView.LANDSCAPE_HIGH_ANGLE)
        )
        self.camera_view_actions[2].triggered.connect(
            lambda: on_camera_view_change(CameraView.LEFT_HALF_HIGH_ANGLE)
        )
        self.camera_view_actions[3].triggered.connect(
            lambda: on_camera_view_change(CameraView.RIGHT_HALF_HIGH_ANGLE)
        )
        for camera_view_action in self.camera_view_actions:
            self.camera_view_menu.addAction(camera_view_action)

        # Setup Measure button for enabling/disabling measure mode
        self.measure_button = QPushButton()
        self.measure_button.setToolTip("Measure")
        self.measure_button.setIcon(icons.get_measure_icon(self.BUTTON_ICON_COLOR))
        self.measure_button.setStyleSheet(self.get_button_style())
        self.measure_button.setShortcut("m")
        self.measure_button.clicked.connect(lambda: on_measure_mode())

        # Setup Help button
        self.help_button = QPushButton()
        self.help_button.setToolTip("Help")
        self.help_button.setIcon(icons.get_help_icon(self.BUTTON_ICON_COLOR))
        self.help_button.setStyleSheet(self.get_button_style())
        self.help_button.clicked.connect(
            lambda: QMessageBox.information(self, "Help", THUNDERSCOPE_HELP_TEXT)
        )

        # Setup pause button
        self.pause_button = QPushButton()
        self.pause_button.setStyleSheet(self.get_button_style())
        self.toggle_pause_button(True)
        # buffer for the simulator pause / play state
        self.simulation_state_buffer = ThreadSafeBuffer(5, SimulationState)

        # Setup Toolbars button for toggling visibility of toolbars
        self.toolbars_button = QPushButton()
        self.toolbars_button.setText("Toolbars")
        self.toolbars_menu = QMenu()
        self.toolbars_menu_checkboxes = {}
        self.toolbars_button.setMenu(toolbars_menu)
        self.toolbars_button.setStyleSheet(self.get_button_style())

        # Setup simulation speed button and menu
        self.sim_speed_menu = QMenu()
        self.sim_speed_button = QPushButton()
        self.sim_speed_button.setText("Speed: 1.00x")
        self.sim_speed_button.setStyleSheet(self.get_button_style())
        self.sim_speed_button.setMenu(self.sim_speed_menu)
        self.sim_speed_button.setToolTip("Simulation Speed")

        # Speed callback should be updated by the parent widget which
        # handles simulation controls
        self.speed_callback = None
        self.simulation_speeds = SIMULATION_SPEEDS
        for speed in self.simulation_speeds:
            self.sim_speed_menu.addAction(
                str(speed),
                lambda new_speed=speed: self.speed_callback(new_speed),
            )

        # if sandbox mode, set up the sandbox control buttons
        if sandbox_mode:
            # Setup Undo button
            self.undo_button = ToggleableButton(False)
            self.undo_button.setToolTip("Undo")
            self.undo_button.setIcon(icons.get_undo_icon(self.BUTTON_ICON_COLOR))
            self.undo_button.setStyleSheet(self.get_button_style(False))

            # Setup Redo button
            self.redo_button = ToggleableButton(False)
            self.redo_button.setToolTip("Redo")
            self.redo_button.setIcon(icons.get_redo_icon(self.BUTTON_ICON_COLOR))
            self.redo_button.setStyleSheet(self.get_button_style(False))

            self.reset_button = QPushButton()
            self.reset_button.setToolTip("Reset")
            self.reset_button.setIcon(icons.get_reset_icon(self.BUTTON_ICON_COLOR))
            self.reset_button.setStyleSheet(self.get_button_style())

        # Setup toolbar
        self.layout().addWidget(self.layers_button)
        self.layout().addWidget(self.toolbars_button)
        self.layout().addStretch()
        if sandbox_mode:
            self.layout().addWidget(self.sim_speed_button)
            self.layout().addWidget(self.reset_button)
            self.layout().addWidget(self.undo_button)
            self.layout().addWidget(self.pause_button)
            self.layout().addWidget(self.redo_button)
        self.layout().addWidget(self.help_button)
        self.layout().addWidget(self.measure_button)
        self.layout().addWidget(self.camera_view_button)

    def refresh(self) -> None:
        """Refreshes the UI for all the toolbar icons and updates toolbar position"""
        # update the pause button state
        simulation_state = self.simulation_state_buffer.get(
            block=False, return_cached=False
        )
        if simulation_state:
            self.toggle_pause_button(simulation_state.is_playing)
            self.update_simulation_speed(simulation_state.simulation_speed)

    def toggle_pause_button(self, is_playing: bool) -> None:
        """Toggles the state of the pause button by updating its text and icon

        :param is_playing: True if the button is in the Play state, False if its in the Pause state
        """
        self.pause_button.setToolTip("Pause" if is_playing else "Play")
        self.pause_button.setIcon(
            icons.get_pause_icon(self.BUTTON_ICON_COLOR)
            if is_playing
            else icons.get_play_icon(self.BUTTON_ICON_COLOR)
        )

    def update_simulation_speed(self, speed: float) -> None:
        """Updates the simulation speed label

        :param speed: the speed of the simulation
        """
        self.sim_speed_button.setText(f"Speed: {speed:.2f}x")

    def toggle_undo_enabled(self, enabled: bool) -> None:
        """Callback function to enable / disable the undo button based on the given state

        :param enabled: if the undo button is enabled or not
        """
        self.undo_button.toggle_enabled(enabled)
        self.undo_button.setStyleSheet(self.get_button_style(enabled))
        self.undo_button.repaint()

    def toggle_redo_enabled(self, enabled: bool) -> None:
        """Callback function to enable / disable the redo button based on the given state

        :param enabled: if the redo button is enabled or not
        """
        self.redo_button.toggle_enabled(enabled)
        self.redo_button.setStyleSheet(self.get_button_style(enabled))
        self.redo_button.repaint()

    def set_speed_callback(self, callback: Callable[[float], None]) -> None:
        """Sets the callback function for updating the simulation speed

        :param callback: the callback function to update the simulation speed
        """
        self.speed_callback = callback


class ShiftButtonToolbar(GLToolbar):
    def __init__(self, parent) -> None:
        super().__init__(parent)
        self.menu = QMenu()
        self.push_button = QPushButton()
        self.push_button.setText("Change Shift Button Behavior")
        button_style = textwrap.dedent(
            f"""
            QPushButton {{
                color: #969696;
                background-color: transparent;
                border-color: transparent;
                icon-size: 22px;
                border-width: 4px;
                border-radius: 4px;
                height: 16px;
            }}
            QPushButton:hover {{
                background-color: {"#363636"};
                border-color: {"#363636"};
            }}
            """
        )
        self.push_button.setStyleSheet(button_style)
        self.push_button.setMenu(self.menu)

        self.actions = [
            QtGui.QAction("[1] Disable Shift Click Ball Move"),
            QtGui.QAction("[2] Enable Shift Click Ball Move"),
        ]

        self.actions[0].triggered.connect(self.disable_ball_movement)
        self.actions[1].triggered.connect(self.enable_ball_movement)

        for action in self.actions:
            self.menu.addAction(action)

        self.layout().addWidget(self.push_button)

    def disable_ball_movement(self):
        print("I have been disable")

    def enable_ball_movement(self):
        print("I have been enabled")


class MultiToolbarLayer(QWidget):
    def __init__(self, parent: QWidget, toolbars: List[GLToolbar]):
        super().__init__(parent)

        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        self.setStyleSheet("background-color: rgba(0,0,0,0);" "padding: 0px;")
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_StyledBackground)
        self.setLayout(QHBoxLayout())

        self.toolbars: List[GLToolbar] = toolbars

        for toolbar in toolbars:
            self.layout().addWidget(toolbar)
            toolbar.hide()

       # Create a shortcut for the F1 key
        self.shortcut_f1 = QtGui.QShortcut(QKeySequence("F1"), self)
        self.shortcut_f1.activated.connect(lambda: self.show_toolbar(0))

        self.shortcut_f1 = QtGui.QShortcut(QKeySequence("F2"), self)
        self.shortcut_f1.activated.connect(lambda: self.show_toolbar(1))

    def add_toolbar(self, toolbar: GLToolbar):
        self.toolbars.append(toolbar)
        self.layout().addWidget(toolbar)
        toolbar.hide()

        self.show_toolbar(0)

    def show_toolbar(self, num):
        # cannot show toolbar, since index is out of range
        if num >= len(self.toolbars):
            return

        for toolbar in self.toolbars: 
            toolbar.hide()

        self.toolbars[num].show()
