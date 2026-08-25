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

# Sandbox status label colors
ON_COLOR = "#00D000"
OFF_COLOR = "#FF0000"


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
        replay_mode: bool = False,
        on_add_bookmark=Callable[[], None],
    ):
        """Set up the toolbar with these buttons:

        - Layers select menu
        - Help
        - Measure Mode Toggle
        - Camera View Select menu
        - Add bookmark

        :param parent: the parent to overlay this toolbar over
        :param on_camera_view_change: the callback function for when the camera view is changed
        :param on_measure_mode: the callback function for when measure mode is toggled
        :param layers_menu: the QMenu for the layers menu selection
        :param toolbars_menu: the QMenu for the toolbars menu selection
        :param replay_mode: if replay mode is enabled
        :param on_add_bookmark: the callback function when adding a bookmark
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
        for idx, camera_view in enumerate(CameraView):
            camera_view_action = QtGui.QAction(f"[{idx}] {camera_view.value}")
            camera_view_action.triggered.connect(
                lambda: on_camera_view_change(camera_view)
            )
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

        # buffer for the simulator state
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

        self.sandbox_sidebar_visible = False
        self.sidebar_visibility_callback = None

        self.sidebar_button_container = QWidget()
        sidebar_button_layout = QHBoxLayout()
        sidebar_button_layout.setContentsMargins(0, 0, 0, 0)
        sidebar_button_layout.setSpacing(4)

        # sandbox mode title
        sidebar_button_layout.addWidget(QLabel("Sandbox Mode"))

        # label to indicate sandbox mode state
        self.sandbox_sidebar_label = QLabel()
        sidebar_button_layout.addWidget(self.sandbox_sidebar_label)

        # button to show sidebar
        self.sidebar_open_button = StyledButton()
        self.sidebar_open_button.setToolTip("Toggle Sandbox Sidebar")
        self.sidebar_open_button.clicked.connect(self.toggle_sidebar_visibility)
        self.__update_sidebar_open_button()
        sidebar_button_layout.addWidget(self.sidebar_open_button)

        self.sidebar_button_container.setLayout(sidebar_button_layout)

        # turn off sandbox mode as default
        self.set_sandbox_mode_enabled(False)

        # Setup toolbar
        self.layout().addWidget(self.layers_button)
        self.add_separator(self.layout())
        self.layout().addWidget(self.toolbars_button)
        self.add_separator(self.layout())
        self.layout().addWidget(self.help_button)
        self.layout().addWidget(self.measure_button)
        self.layout().addWidget(self.camera_view_button)

        if not replay_mode:
            self.layout().addWidget(self.bookmark_button)

        self.layout().addStretch()
        self.layout().addWidget(self.sidebar_button_container)

    @override
    def refresh(self) -> None:
        """Updates the sim speed"""
        simulation_state = self.simulation_state_buffer.get(
            block=False, return_cached=False
        )
        if simulation_state:
            self.update_simulation_speed(simulation_state.simulation_speed)

        self.setGeometry(0, 0, self.parentWidget().width(), self.height())

    def update_simulation_speed(self, speed: float) -> None:
        """Updates the simulation speed label

        :param speed: the speed of the simulation
        """
        self.sim_speed_button.setText(f"Speed: {speed:.2f}x")

    def set_speed_callback(self, callback: Callable[[float], None]) -> None:
        """Sets the callback function for updating the simulation speed

        :param callback: the callback function to update the simulation speed
        """
        self.speed_callback = callback

    def set_sidebar_visibility_callback(
        self, callback: Callable[[float], None]
    ) -> None:
        """Sets the callback function for toggling sidebar visibility

        :param callback: the callback function for toggling sidebar visibility
        """
        self.sidebar_visibility_callback = callback

    def toggle_sidebar_visibility(self) -> None:
        """Toggles the sandbox sidebar visibility between shown and hidden

        Flips the internal visibility state, updates the sidebar open button
        icon to reflect the new state, and invokes the sidebar visibility
        callback if one has been set.
        """
        self.sandbox_sidebar_visible = not self.sandbox_sidebar_visible

        self.__update_sidebar_open_button()

        if self.sidebar_visibility_callback:
            self.sidebar_visibility_callback(self.sandbox_sidebar_visible)

    def __update_sidebar_open_button(self) -> None:
        """Updates the sidebar open button icon to reflect current visibility

        Shows an up icon when the sidebar is visible, and a
        down icon when it is hidden.
        """
        self.sidebar_open_button.setIcon(
            qta.icon(
                (
                    "fa6s.chevron-up"
                    if self.sandbox_sidebar_visible
                    else "fa6s.chevron-down"
                ),
                color=self.BUTTON_ICON_COLOR,
            )
        )

    def set_sandbox_mode_enabled(self, enabled: bool) -> None:
        """Sets the sandbox enabled state and updates the label

        :param enabled: whether sandbox mode is enabled
        """
        self.sandbox_enabled = enabled
        color = ON_COLOR if enabled else OFF_COLOR
        self.sandbox_sidebar_label.setText("On" if enabled else "Off")
        self.sandbox_sidebar_label.setStyleSheet(f"color: {color}; font-weight: bold;")
