from typing import Callable, override
from pyqtgraph.Qt import QtGui
from pyqtgraph.Qt.QtWidgets import *
from proto.import_all_protos import *
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.constants import (
    CameraView,
    THUNDERSCOPE_HELP_TEXT,
    SIMULATION_SPEEDS,
)
from software.thunderscope.common.common_widgets import (
    StyledButton,
)
from software.thunderscope.gl.toolbars.gl_toolbar import GLToolbar
import qtawesome as qta


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
        on_toggle_sidebar: Callable[[], None],
        layers_menu: QMenu,
        toolbars_menu: QMenu,
        sandbox_mode: bool = False,
        replay_mode: bool = False,
        on_add_bookmark=Callable[[], None],
    ):
        """Set up the toolbar with these buttons:

        - Layers select menu
        - Undo
        - Pause
        - Redo
        - Help
        - Measure Mode Toggle
        - Camera View Select menu
        - Add bookmark
        - Toggle sandbox sidebar

        :param parent: the parent to overlay this toolbar over
        :param on_camera_view_change: the callback function for when the camera view is changed
        :param on_measure_mode: the callback function for when measure mode is toggled
        :param layers_menu: the QMenu for the layers menu selection
        :param toolbars_menu: the QMenu for the toolbars menu selection
        :param replay_mode: if replay mode is enabled
        :param on_add_bookmark: the callback function when adding a bookmark
        :param on_toggle_sidebar: the callback function when toggling the sandbox sidebar
        """
        super(GLFieldToolbar, self).__init__(parent=parent)

        # Setup Layers button for toggling visibility of layers
        self.layers_button = StyledButton()
        self.layers_button.setText("Layers")
        self.layers_button.setMenu(layers_menu)

        # Set up View button for setting the camera position to standard views
        self.camera_view_button = StyledButton()
        self.camera_view_button.setToolTip("View")
        self.camera_view_button.setIcon(
            qta.icon("msc.device-camera-video", color=self.BUTTON_ICON_COLOR)
        )
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
        self.measure_button = StyledButton()
        self.measure_button.setToolTip("Measure")
        self.measure_button.setIcon(
            qta.icon("ph.ruler-light", color=self.BUTTON_ICON_COLOR)
        )
        self.measure_button.setShortcut("m")
        self.measure_button.clicked.connect(lambda: on_measure_mode())

        # Setup Help button
        self.help_button = StyledButton()
        self.help_button.setToolTip("Help")
        self.help_button.setIcon(
            qta.icon("mdi.help-circle", color=self.BUTTON_ICON_COLOR)
        )
        self.help_button.clicked.connect(
            lambda: QMessageBox.information(
                self.window(), "Help", THUNDERSCOPE_HELP_TEXT
            )
        )

        # Setup pause button
        self.pause_button = StyledButton()
        self.toggle_pause_button(True)
        # buffer for the simulator pause / play state
        self.simulation_state_buffer = ThreadSafeBuffer(5, SimulationState)

        # Setup Toolbars button for toggling visibility of toolbars
        self.toolbars_button = StyledButton()
        self.toolbars_button.setText("Toolbars")
        self.toolbars_menu = QMenu()
        self.toolbars_menu_checkboxes = {}
        self.toolbars_button.setMenu(toolbars_menu)

        if not replay_mode:
            self.bookmark_button = StyledButton()
            self.bookmark_button.setIcon(
                qta.icon(("fa6.bookmark"), color=self.BUTTON_ICON_COLOR)
            )
            self.bookmark_button.setShortcut("b")
            self.bookmark_button.clicked.connect(on_add_bookmark)

        # Setup simulation speed button and menu
        self.sim_speed_menu = QMenu()
        self.sim_speed_button = StyledButton()
        self.sim_speed_button.setText("Speed: 1.00x")
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

        self.sandbox_sidebar_button = StyledButton()
        self.sandbox_sidebar_button.setToolTip("Toggle Sandbox Sidebar")
        self.sandbox_sidebar_button.setIcon(
            qta.icon("mdi.view-sidebar-outline", color=self.BUTTON_ICON_COLOR)
        )
        self.sandbox_sidebar_button.clicked.connect(self.on_toggle_sidebar)

        # Setup toolbar
        self.layout().addWidget(self.layers_button)
        self.layout().addWidget(self.toolbars_button)
        self.layout().addStretch()
        self.layout().addWidget(self.help_button)
        self.layout().addWidget(self.measure_button)
        self.layout().addWidget(self.camera_view_button)
        self.layout().addWidget(self.sandbox_sidebar_button)

        if not replay_mode:
            self.layout().addWidget(self.bookmark_button)

    @override
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
            qta.icon(
                "fa6s.pause" if is_playing else "fa5s.play",
                color=self.BUTTON_ICON_COLOR,
            )
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
        self.undo_button.repaint()

    def toggle_redo_enabled(self, enabled: bool) -> None:
        """Callback function to enable / disable the redo button based on the given state

        :param enabled: if the redo button is enabled or not
        """
        self.redo_button.toggle_enabled(enabled)
        self.redo_button.repaint()

    def set_speed_callback(self, callback: Callable[[float], None]) -> None:
        """Sets the callback function for updating the simulation speed

        :param callback: the callback function to update the simulation speed
        """
        self.speed_callback = callback

    def set_sandbox_toggle_callback(self, callback: Callable[[bool], None]) -> None:
        """Sets the callback function for toggling sandbox mode

        :param callback: the callback function to toggle sandbox mode
        """
        self.sandbox_toggle_callback = callback
