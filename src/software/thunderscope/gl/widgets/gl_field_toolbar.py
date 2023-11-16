import textwrap
from typing import Callable
from pyqtgraph.Qt import QtGui, QtCore
from pyqtgraph.Qt.QtWidgets import *
from software.thunderscope.constants import CameraView, THUNDERSCOPE_HELP_TEXT
import software.thunderscope.gl.widgets.toolbar_icons.toolbar_icon_loader as icons


class GLFieldToolbar(QWidget):
    """
    Toolbar for the GL Field Widget

    Has buttons for measure mode, changing camera views, showing help info
    And for undoing / redoing robot state changes
    """

    BUTTON_ICON_COLOR = "white"

    def __init__(
        self,
        on_camera_view_change: Callable[[CameraView], None],
        on_measure_mode: Callable[[], None],
        layers_menu: QMenu,
    ):
        """
        Set up the toolbar with these buttons:

        - Layers select menu
        - Undo
        - Pause
        - Redo
        - Help
        - Measure Mode Toggle
        - Camera View Select menu

        :param on_camera_view_change: the callback function for when the camera view is changed
        :param on_measure_mode: the callback function for when measure mode is toggled
        :param layers_menu: the QMenu for the layers menu selection
        """
        super(GLFieldToolbar, self).__init__()

        # Setup Layers button for toggling visibility of layers
        self.layers_button = QPushButton()
        self.layers_button.setText("Layers")
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
        self.pause_button.setToolTip("Pause")
        self.pause_button.setIcon(icons.get_pause_icon(self.BUTTON_ICON_COLOR))
        self.pause_button.setStyleSheet(self.get_button_style())

        # Setup Undo button
        self.undo_button = QPushButton()
        self.undo_button.setToolTip("Undo")
        self.undo_button.setIcon(icons.get_undo_icon(self.BUTTON_ICON_COLOR))
        self.undo_button.setStyleSheet(self.get_button_style(False))

        # Setup Redo button
        self.redo_button = QPushButton()
        self.redo_button.setToolTip("Redo")
        self.redo_button.setIcon(icons.get_redo_icon(self.BUTTON_ICON_COLOR))
        self.redo_button.setStyleSheet(self.get_button_style(False))

        self.reset_button = QPushButton()
        self.reset_button.setToolTip("Reset")
        self.reset_button.setIcon(icons.get_reset_icon(self.BUTTON_ICON_COLOR))
        self.reset_button.setStyleSheet(self.get_button_style())

        # Setup toolbar
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        self.setStyleSheet("background-color: black;" "padding: 0px;")
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_StyledBackground)
        self.setLayout(QHBoxLayout())
        self.layout().addWidget(self.layers_button)
        self.layout().addStretch()
        self.layout().addWidget(self.reset_button)
        self.layout().addWidget(self.undo_button)
        self.layout().addWidget(self.pause_button)
        self.layout().addWidget(self.redo_button)
        self.layout().addWidget(self.help_button)
        self.layout().addWidget(self.measure_button)
        self.layout().addWidget(self.camera_view_button)

    def toggle_pause_button_text(self, is_playing: bool):
        self.pause_button.setText("Pause" if is_playing else "Play")

    def get_button_style(self, is_enabled: bool = True):
        # the style for each toolbar button
        return textwrap.dedent(
            f"""
            QPushButton {{
                background-color: transparent;
                border-color: transparent;
                icon-size: 22px;
                border-width: 4px;
                border-radius: 4px;
                height: 16px;
            }}
            QPushButton:hover {{
                background-color: {"#363636" if is_enabled else "transparent"};
                border-color: {"#363636" if is_enabled else "transparent"};
            }}
            """
        )

    def toggle_undo_enabled(self, enabled: bool) -> None:
        """
        Callback function to enable / disable the undo button based on the given state
        :param enabled: if the undo button is enabled or not
        """
        self.undo_button.setStyleSheet(self.get_button_style(enabled))
        self.undo_button.repaint()

    def toggle_redo_enabled(self, enabled: bool) -> None:
        """
        Callback function to enable / disable the redo button based on the given state
        :param enabled: if the redo button is enabled or not
        """
        self.redo_button.setStyleSheet(self.get_button_style(enabled))
        self.redo_button.repaint()
