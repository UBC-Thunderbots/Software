import textwrap
from typing import Callable
from pyqtgraph.Qt import QtGui
from pyqtgraph.Qt.QtWidgets import *
from software.thunderscope.constants import CameraView, THUNDERSCOPE_HELP_TEXT


class GLFieldToolbar(QWidget):
    """
    Toolbar for the GL Field Widget

    Has buttons for measure mode, changing camera views, showing help info
    And for undoing / redoing robot state changes
    """

    def __init__(self, on_camera_view_change: Callable[[CameraView], None], on_measure_mode: Callable[[], None], layers_menu: QMenu):
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
        self.camera_view_button.setText("View")
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
        self.measure_button.setText("Measure")
        self.measure_button.setStyleSheet(self.get_button_style())
        self.measure_button.setShortcut("m")
        self.measure_button.clicked.connect(lambda: on_measure_mode())

        # Setup Help button
        self.help_button = QPushButton()
        self.help_button.setText("Help")
        self.help_button.setStyleSheet(self.get_button_style())
        self.help_button.clicked.connect(
            lambda: QMessageBox.information(self, "Help", THUNDERSCOPE_HELP_TEXT)
        )

        # Setup play button
        self.play_button = QPushButton()
        self.play_button.setText("Pause")
        self.play_button.setStyleSheet(self.get_button_style())

        # Setup Undo button
        self.undo_button = QPushButton()
        self.undo_button.setText("Undo")
        self.undo_button.setStyleSheet(self.get_button_style())

        # Setup Redo button
        self.redo_button = QPushButton()
        self.redo_button.setText("Redo")
        self.redo_button.setStyleSheet(self.get_button_style())

        self.reset_button = QPushButton()
        self.reset_button.setText("Reset")
        self.reset_button.setStyleSheet(self.get_button_style())

        # Setup toolbar
        self.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed
        )
        self.setStyleSheet("background-color: black;" "padding: 0px;")
        self.setLayout(QHBoxLayout())
        self.layout().addWidget(self.layers_button)
        self.layout().addStretch()
        self.layout().addWidget(self.reset_button)
        self.layout().addWidget(self.undo_button)
        self.layout().addWidget(self.play_button)
        self.layout().addWidget(self.redo_button)
        self.layout().addWidget(self.help_button)
        self.layout().addWidget(self.measure_button)
        self.layout().addWidget(self.camera_view_button)

    def toggle_play_button_text(self, is_playing: bool):
        self.play_button.setText("Pause" if is_playing else "Play")

    def get_button_style(self, is_enabled: bool = True):
        # the style for each toolbar button
        return textwrap.dedent(
            f"""
            QPushButton {{
                color: {"#969696" if is_enabled else "#474747"};
                background-color: transparent;
                border-color: transparent;
                border-width: 4px;
                border-radius: 4px;
                height: 16px;
            }}
            QPushButton:hover {{
                background-color: #363636;
                border-color: #363636;
            }}
            """
        )