from typing import Callable, override
from pyqtgraph.Qt import QtCore
from pyqtgraph.Qt.QtWidgets import *
from proto.import_all_protos import *
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
import qtawesome as qta
from software.thunderscope.common.common_widgets import ToggleableButton, StyledButton
from software.thunderscope.constants import SANDBOX_MODE_HELP_TEXT


class GLSandboxSidebar(QWidget):
    """Sidebar widget for the sandbox mode

    Absolutely positioned to the right of its parent, overlaying on top,
    taking the full vertical space. Uses a QVBoxLayout to arrange sandbox
    controls vertically.
    """

    CONTENT_COLOR = "white"
    BACKGROUND_COLOR = "black"
    POSITION_PADDING_MULTIPLIER = 0.1
    SIDEBAR_WIDTH_RATIO = 0.2

    def __init__(self, parent: QWidget, widget_above: QWidget):
        """Set up the sandbox sidebar

        :param parent: the parent widget to attach this sidebar to
        :param widget_above: the widget above the sidebar for placement
        """
        super().__init__(parent=parent)
        self.widget_above = widget_above

        # Setup sidebar with a vertical layout
        self.setLayout(QVBoxLayout())

        self.sidebar_enabled = False
        self.sidebar_rendered = False
        self.sandbox_mode_enabled = False
        self._sandbox_mode_callbacks: list[Callable[[bool], None]] = []

        # Create a container widget to hold all sidebar contents
        self.sidebar_container = QWidget()
        self.sidebar_container.setLayout(QVBoxLayout())
        self.sidebar_container.setObjectName("sidebarContainer")

        # Style with a dark background + border so it's visible when overlaying
        self.sidebar_container.setStyleSheet(
            f"#sidebarContainer {{"
            f"    background-color: {self.BACKGROUND_COLOR};"
            f"    border: 2px solid {self.CONTENT_COLOR};"
            f"    border-radius: 5px;"
            f"    padding: 15px 10px;"
            f"    padding-top: 10px;"
            f"}}"
        )
        self.sidebar_container.setAttribute(
            QtCore.Qt.WidgetAttribute.WA_StyledBackground
        )
        self.sidebar_container.setSizePolicy(
            QSizePolicy.Policy.MinimumExpanding, QSizePolicy.Policy.Preferred
        )

        # Setup sandbox mode toggle checkbox
        self.checkbox_layout = QHBoxLayout()
        self.checkbox_layout.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.checkbox_layout.addWidget(QLabel("Enable Sandbox Mode: "))

        self.sandbox_mode_checkbox = QCheckBox()
        self.sandbox_mode_checkbox.setChecked(False)
        self.sandbox_mode_checkbox.stateChanged.connect(self.toggle_sandbox_mode)
        self.checkbox_layout.addWidget(self.sandbox_mode_checkbox)
        self.sidebar_container.layout().addStretch()
        self.sidebar_container.layout().addLayout(self.checkbox_layout)

        self.help_layout = QHBoxLayout()
        self.help_layout.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.help_layout.addWidget(QLabel("How to Use: "))

        self.help_button = StyledButton()
        self.help_button.setToolTip("Help")
        self.help_button.setIcon(qta.icon("mdi6.help-circle", color=self.CONTENT_COLOR))
        self.help_layout.addWidget(self.help_button)
        self.help_button.clicked.connect(
            lambda: QMessageBox.information(
                self.window(), "Help", SANDBOX_MODE_HELP_TEXT
            )
        )
        self.sidebar_container.layout().addStretch()
        self.sidebar_container.layout().addLayout(self.help_layout)

        # Setup pause button
        self.pause_button = ToggleableButton(False)
        self.toggle_pause_button(True)
        # buffer for the simulator pause / play state
        self.simulation_state_buffer = ThreadSafeBuffer(5, SimulationState)

        # Setup Undo button
        self.undo_button_enabled = False
        self.undo_button = ToggleableButton(False)
        self.undo_button.setToolTip("Undo")
        self.undo_button.setIcon(
            qta.icon("mdi6.undo-variant", color=self.CONTENT_COLOR)
        )

        # Setup Redo button
        self.redo_button_enabled = False
        self.redo_button = ToggleableButton(False)
        self.redo_button.setToolTip("Redo")
        self.redo_button.setIcon(
            qta.icon("mdi6.redo-variant", color=self.CONTENT_COLOR)
        )

        button_layout = QHBoxLayout()
        button_layout.addWidget(self.pause_button)
        button_layout.addWidget(self.undo_button)
        button_layout.addWidget(self.redo_button)
        self.sidebar_container.layout().addStretch()
        self.sidebar_container.layout().addLayout(button_layout)

        # Setup Clear Field button
        self.clear_field_button = ToggleableButton(False)
        self.clear_field_button.setToolTip("Clears all robots from the field")
        self.clear_field_button.setText("Clear Field")
        self.sidebar_container.layout().addStretch()
        self.sidebar_container.layout().addWidget(self.clear_field_button)

        self.sidebar_container.layout().addStretch()

        # Add the container to the main layout, with stretch underneath
        self.layout().addWidget(self.sidebar_container)
        self.layout().addStretch()

        self.hide()

        # Listen for parent resize events to reposition
        parent.installEventFilter(self)

    def refresh(self) -> None:
        """Refreshes the UI to move the sidebar and update the pause button state"""
        if not self.sidebar_rendered:
            if self.sidebar_enabled:
                self.reposition()
                self.show()
            else:
                self.hide()
            self.sidebar_rendered = True

        # update the pause button state
        simulation_state = self.simulation_state_buffer.get(
            block=False, return_cached=False
        )
        if simulation_state:
            self.toggle_pause_button(simulation_state.is_playing)

    def register_sandbox_mode_callback(self, callback: Callable[[bool], None]) -> None:
        """Register a callback that will be called when sandbox mode is toggled.

        :param callback: A callable that takes one boolean argument (the new state).
        """
        self._sandbox_mode_callbacks.append(callback)

    def toggle_sandbox_mode(self) -> None:
        """Toggle sandbox mode on/off and notify all registered callbacks."""
        self.sandbox_mode_enabled = not self.sandbox_mode_enabled
        for callback in self._sandbox_mode_callbacks:
            callback(self.sandbox_mode_enabled)

        self.pause_button.toggle_enabled(self.sandbox_mode_enabled)
        self.clear_field_button.toggle_enabled(self.sandbox_mode_enabled)

        self.undo_button.toggle_enabled(
            self.undo_button_enabled and self.sandbox_mode_enabled
        )
        self.redo_button.toggle_enabled(
            self.redo_button_enabled and self.sandbox_mode_enabled
        )

    def toggle_pause_button(self, is_playing: bool) -> None:
        """Toggles the state of the pause button by updating its text and icon

        :param is_playing: True if the button is in the Play state, False if its in the Pause state
        """
        self.pause_button.setToolTip("Pause" if is_playing else "Play")
        self.pause_button.setIcon(
            qta.icon(
                "fa6s.pause" if is_playing else "fa5s.play",
                color=self.CONTENT_COLOR,
            )
        )

    def toggle_visibility(self, enabled: bool):
        """Toggle the sidebar visibility"""
        self.sidebar_enabled = enabled
        self.sidebar_rendered = False

    def reposition(self):
        """Position to the right edge of the parent, taking full height"""
        parent = self.parent()
        width = int(parent.width() * self.SIDEBAR_WIDTH_RATIO)
        self.setGeometry(
            parent.geometry().right() - width,
            self.widget_above.height(),
            width,
            parent.height(),
        )

    @override
    def eventFilter(self, obj, event):
        """Reposition when the parent is resized"""
        if obj is self.parent() and event.type() == QtCore.QEvent.Resize:
            self.reposition()
        return super().eventFilter(obj, event)

    def toggle_undo_enabled(self, enabled: bool) -> None:
        """Callback function to enable / disable the undo button based on the given state

        :param enabled: if the undo button is enabled or not
        """
        self.undo_button_enabled = enabled
        self.undo_button.toggle_enabled(enabled)
        self.undo_button.repaint()

    def toggle_redo_enabled(self, enabled: bool) -> None:
        """Callback function to enable / disable the redo button based on the given state

        :param enabled: if the redo button is enabled or not
        """
        self.redo_button_enabled = enabled
        self.redo_button.toggle_enabled(enabled)
        self.redo_button.repaint()
