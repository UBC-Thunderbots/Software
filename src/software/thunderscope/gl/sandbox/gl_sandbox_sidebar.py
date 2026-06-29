from typing import Callable, override
from pyqtgraph.Qt.QtCore import QEvent
from pyqtgraph.Qt.QtWidgets import *

import qtawesome as qta
from software.thunderscope.common.common_widgets import ToggleableButton


class GLSandboxSidebar(QWidget):
    """Sidebar widget for the sandbox mode

    Absolutely positioned to the right of its parent, overlaying on top,
    taking the full vertical space. Uses a QVBoxLayout to arrange sandbox
    controls vertically.
    """

    BUTTON_ICON_COLOR = "white"
    SIDEBAR_WIDTH_RATIO = 0.3

    def __init__(
        self,
        parent: QWidget,
        on_sandbox_mode_toggle: Callable[[bool], None] = None,
    ):
        """Set up the sandbox sidebar

        :param parent: the parent widget to attach this sidebar to
        :param on_sandbox_mode_toggle: callback when sandbox mode is toggled on/off
        """
        super().__init__(parent=parent)

        # Setup sidebar with a vertical layout
        self.setLayout(QVBoxLayout())

        self.sidebar_enabled = False

        # Style with a dark background so it's visible when overlaying
        self.setStyleSheet(
            "background-color: rgba(30, 30, 30, 220);"
            "border-left: 1px solid #555;"
        )

        # Setup sandbox mode toggle checkbox
        self.sandbox_mode_checkbox = QCheckBox("Enable Sandbox Mode")
        self.sandbox_mode_checkbox.setChecked(False)
        self.sandbox_mode_checkbox.stateChanged.connect(
            lambda checked: on_sandbox_mode_toggle(
                self.sandbox_mode_checkbox.isChecked()
            )
            if on_sandbox_mode_toggle
            else None
        )
        self.layout().addWidget(self.sandbox_mode_checkbox)

        # Setup Undo button
        self.undo_button = ToggleableButton(False)
        self.undo_button.setToolTip("Undo")
        self.undo_button.setIcon(
            qta.icon("mdi6.undo-variant", color=self.BUTTON_ICON_COLOR)
        )

        # Setup Redo button
        self.redo_button = ToggleableButton(False)
        self.redo_button.setToolTip("Redo")
        self.redo_button.setIcon(
            qta.icon("mdi6.redo-variant", color=self.BUTTON_ICON_COLOR)
        )

        button_layout = QHBoxLayout()
        button_layout.addWidget(self.undo_button)
        button_layout.addWidget(self.redo_button)
        self.layout().addLayout(button_layout)

        # Setup Clear Field button
        self.clear_field_button = ToggleableButton(False)
        self.clear_field_button.setToolTip("Clear Field")
        self.clear_field_button.setIcon(
            qta.icon("mdi6.delete-variant", color=self.BUTTON_ICON_COLOR)
        )
        self.layout().addWidget(self.clear_field_button)

        # Setup Create new Test button
        self.create_test_button = ToggleableButton(False)
        self.create_test_button.setToolTip("Create new Test")
        self.create_test_button.setIcon(
            qta.icon("mdi6.flask-outline", color=self.BUTTON_ICON_COLOR)
        )
        self.layout().addWidget(self.create_test_button)

        # Setup Add Case to Existing Test button
        self.add_case_button = ToggleableButton(False)
        self.add_case_button.setToolTip("Add Case to Existing Test")
        self.add_case_button.setIcon(
            qta.icon("mdi6.test-tube", color=self.BUTTON_ICON_COLOR)
        )
        self.layout().addWidget(self.add_case_button)

        self.layout().addStretch()

        self.hide()

        # Listen for parent resize events to reposition
        parent.installEventFilter(self)

    def toggle_visibility(self):
        """Toggle the sidebar visibility"""
        self.sidebar_enabled = not self.sidebar_enabled
        if self.sidebar_enabled:
            self.reposition()
            self.show()
        else:
            self.hide()

    def reposition(self):
        """Position to the right edge of the parent, taking full height"""
        parent = self.parent()
        if parent:
            width = int(parent.width() * self.SIDEBAR_WIDTH_RATIO)
            self.setGeometry(
                parent.width() - width,
                0,
                width,
                parent.height(),
            )

    @override
    def eventFilter(self, obj, event):
        """Reposition when the parent is resized"""
        if obj is self.parent() and event.type() == QEvent.Resize:
            self.reposition()
        return super().eventFilter(obj, event)
