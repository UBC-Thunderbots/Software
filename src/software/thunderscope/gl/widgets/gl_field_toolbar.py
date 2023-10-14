from pyqtgraph.Qt import QtGui
from pyqtgraph.Qt.QtWidgets import *
from software.thunderscope.constants import CameraView, THUNDERSCOPE_HELP_TEXT
import textwrap


class GLFieldToolbar(QWidget):
    """
    Toolbar for the GL Field Widget

    Has buttons for measure mode, changing camera views, showing help info
    And for undoing / redoing robot state changes
    """

    # the style for each toolbar button
    TOOL_BUTTON_STYLESHEET = textwrap.dedent(
        """
        QPushButton {
            color: #969696;
            background-color: transparent;
            border-color: transparent;
            border-width: 4px;
            border-radius: 4px;
            height: 16px;
        }
        QPushButton:hover {
            background-color: #363636;
            border-color: #363636;
        }
        """
    )
    def __init__(self):
        super(GLFieldToolbar, self).__init__()

        # Setup Layers button for toggling visibility of layers
        self.layers_button = QPushButton()
        self.layers_button.setText("Layers")
        self.layers_button.setStyleSheet(self.TOOL_BUTTON_STYLESHEET)

        # Set up View button for setting the camera position to standard views
        self.camera_view_button = QPushButton()
        self.camera_view_button.setText("View")
        self.camera_view_button.setStyleSheet(self.TOOL_BUTTON_STYLESHEET)
        self.camera_view_menu = QMenu()
        self.camera_view_button.setMenu(self.camera_view_menu)
        self.camera_view_actions = [
            QtGui.QAction("[1] Orthographic Top Down"),
            QtGui.QAction("[2] Landscape High Angle"),
            QtGui.QAction("[3] Left Half High Angle"),
            QtGui.QAction("[4] Right Half High Angle"),
        ]
        self.camera_view_actions[0].triggered.connect(
            lambda: self.set_camera_view(CameraView.ORTHOGRAPHIC)
        )
        self.camera_view_actions[1].triggered.connect(
            lambda: self.set_camera_view(CameraView.LANDSCAPE_HIGH_ANGLE)
        )
        self.camera_view_actions[2].triggered.connect(
            lambda: self.set_camera_view(CameraView.LEFT_HALF_HIGH_ANGLE)
        )
        self.camera_view_actions[3].triggered.connect(
            lambda: self.set_camera_view(CameraView.RIGHT_HALF_HIGH_ANGLE)
        )
        for camera_view_action in self.camera_view_actions:
            self.camera_view_menu.addAction(camera_view_action)

        # Setup Measure button for enabling/disabling measure mode
        self.measure_mode_enabled = False
        self.measure_layer = None
        self.measure_button = QPushButton()
        self.measure_button.setText("Measure")
        self.measure_button.setStyleSheet(self.TOOL_BUTTON_STYLESHEET)
        self.measure_button.setShortcut("m")

        # Setup Help button
        self.help_button = QPushButton()
        self.help_button.setText("Help")
        self.help_button.setStyleSheet(self.TOOL_BUTTON_STYLESHEET)
        self.help_button.clicked.connect(
            lambda: QMessageBox.information(self, "Help", THUNDERSCOPE_HELP_TEXT)
        )

        # Setup play button
        self.play_button = QPushButton()
        self.play_button.setText("Pause")
        self.play_button.setStyleSheet(self.TOOL_BUTTON_STYLESHEET)
        self.play_button.clicked.connect(
            lambda: self.play_button.setText("Pause" if self.toggle_play_state() else "Play")
        )

        # Setup Undo button
        self.undo_button = QPushButton()
        self.undo_button.setText("Undo")
        self.undo_button.setStyleSheet(self.TOOL_BUTTON_STYLESHEET)

        # Setup Redo button
        self.redo_button = QPushButton()
        self.redo_button.setText("Redo")
        self.redo_button.setStyleSheet(self.TOOL_BUTTON_STYLESHEET)

        # Setup toolbar
        self.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed
        )
        self.setStyleSheet("background-color: black;" "padding: 0px;")
        self.setLayout(QHBoxLayout())
        self.layout().addWidget(self.layers_button)
        self.layout().addStretch()
        self.layout().addWidget(self.undo_button)
        self.layout().addWidget(self.play_button)
        self.layout().addWidget(self.redo_button)
        self.layout().addWidget(self.help_button)
        self.layout().addWidget(self.measure_button)
        self.layout().addWidget(self.camera_view_button)