import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.opengl import *

import textwrap
import numpy as np

from software.thunderscope.constants import *

from software.thunderscope.gl.gl_layer import GLLayer
from software.thunderscope.replay.replay_controls import ReplayControls
from software.thunderscope.gl.helpers.graphics_view import GraphicsView


class GLWidget(QWidget):
    """Widget that handles GLLayers to produce a 3D visualization of the field/world 
    and our AI. GLWidget can also provide replay controls.
    """

    def __init__(self, player=None):
        """Initialize the GLWidget

        :param player: The replay player to optionally display media controls for

        """
        QVBoxLayout.__init__(self)

        # Setup the GraphicsView containing the GLViewWidget
        self.graphics_view = GraphicsView()
        self.graphics_view.gl_view_widget.point_in_scene_picked_signal.connect(
            self.pointInScenePickedEvent
        )

        self.setFocusPolicy(QtCore.Qt.FocusPolicy.StrongFocus)
        self.graphics_view.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)

        # Setup legend
        self.legend = pg.LegendItem((80, 60))
        self.graphics_view.addItem(self.legend)

        # Stylesheet for toolbar buttons
        tool_button_stylesheet = textwrap.dedent(
            """
            QToolButton {
                color: #969696;
                background-color: transparent;
                border-color: transparent;
                border-width: 4px;
                border-radius: 4px;
                height: 16px;
            }
            QToolButton:hover {
                background-color: #363636;
                border-color: #363636;
            }
            """
        )

        # Set up View button for setting the camera position to standard views
        self.camera_view_button = QToolButton()
        self.camera_view_button.setText("View")
        self.camera_view_button.setStyleSheet(tool_button_stylesheet)
        self.camera_view_menu = QtGui.QMenu()
        self.camera_view_button.setMenu(self.camera_view_menu)
        self.camera_view_button.setPopupMode(
            QToolButton.ToolButtonPopupMode.InstantPopup
        )
        self.camera_view_actions = [
            QtGui.QAction("[1] Orthographic Top Down"),
            QtGui.QAction("[2] Landscape High Angle"),
            QtGui.QAction("[3] Left Half High Angle"),
            QtGui.QAction("[4] Right Half High Angle"),
        ]
        self.camera_view_actions[0].triggered.connect(
            lambda: self.setCameraView(CameraView.ORTHOGRAPHIC)
        )
        self.camera_view_actions[1].triggered.connect(
            lambda: self.setCameraView(CameraView.LANDSCAPE_HIGH_ANGLE)
        )
        self.camera_view_actions[2].triggered.connect(
            lambda: self.setCameraView(CameraView.LEFT_HALF_HIGH_ANGLE)
        )
        self.camera_view_actions[3].triggered.connect(
            lambda: self.setCameraView(CameraView.RIGHT_HALF_HIGH_ANGLE)
        )
        for camera_view_action in self.camera_view_actions:
            self.camera_view_menu.addAction(camera_view_action)

        # Setup Help button
        self.help_button = QToolButton()
        self.help_button.setText("Help")
        self.help_button.setStyleSheet(tool_button_stylesheet)
        self.help_button.clicked.connect(
            lambda: QMessageBox.information(self, "Help", THUNDERSCOPE_HELP_TEXT)
        )

        # Setup toolbar
        self.toolbar = QWidget()
        self.toolbar.setMaximumHeight(40)
        self.toolbar.setStyleSheet("background-color: black;" "padding: 0px;")
        self.toolbar.setLayout(QHBoxLayout())
        self.toolbar.layout().addStretch()
        self.toolbar.layout().addWidget(self.help_button)
        self.toolbar.layout().addWidget(self.camera_view_button)

        # Setup layout
        self.layout = QVBoxLayout()
        self.layout.setSpacing(0)
        self.setLayout(self.layout)
        self.layout.addWidget(self.toolbar)
        self.layout.addWidget(self.graphics_view.gl_view_widget)

        # Setup replay controls if player is provided and the log has some size
        self.player = player
        if self.player and self.player.end_time != 0.0:
            self.replay_controls = ReplayControls(player=player)
            self.replay_controls.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
            self.layout.addWidget(self.replay_controls)
        else:
            self.player = None

        # Variables for keeping track of keys pressed
        self.key_pressed = {}
        self.accepted_keys = [Qt.Key.Key_1, Qt.Key.Key_2, Qt.Key.Key_3, Qt.Key.Key_4]
        for key in self.accepted_keys:
            self.key_pressed[key] = False

        self.layers = []

        self.setCameraView(CameraView.LANDSCAPE_HIGH_ANGLE)

    def keyPressEvent(self, event):
        """Detect when a key has been pressed
        
        :param event: The event
        
        """
        self.key_pressed[event.key()] = True

        # Camera view shortcuts
        if self.key_pressed[Qt.Key.Key_1]:
            self.setCameraView(CameraView.ORTHOGRAPHIC)
        elif self.key_pressed[Qt.Key.Key_2]:
            self.setCameraView(CameraView.LANDSCAPE_HIGH_ANGLE)
        elif self.key_pressed[Qt.Key.Key_3]:
            self.setCameraView(CameraView.LEFT_HALF_HIGH_ANGLE)
        elif self.key_pressed[Qt.Key.Key_4]:
            self.setCameraView(CameraView.RIGHT_HALF_HIGH_ANGLE)

        # Propagate keypress event to all layers
        for layer in self.layers:
            layer.keyPressEvent(event)

    def keyReleaseEvent(self, event):
        """Detect when a key has been released
        
        :param event: The event
        
        """
        self.key_pressed[event.key()] = False

        # Propagate keypress event to all layers
        for layer in self.layers:
            layer.keyReleaseEvent(event)

    def pointInScenePickedEvent(self, event):
        """Propagate PointInScenePickedEvent to all layers
        
        :param event: The event
        
        """
        for layer in self.layers:
            layer.pointInScenePickedEvent(event)

    def addLayer(self, name: str, layer: GLLayer, visible: bool = True):
        """Add a layer to this GLWidget and to the legend
        
        :param name: The name of the layer
        :param layer: The GLLayer 
        :param visible: Whether the layer is visible on startup

        """
        self.layers.append(layer)
        self.legend.addItem(layer, name)
        if not visible:
            layer.hide()

    def refresh(self):
        """Trigger an update on all the layers, adding/removing GLGraphicsItem 
        returned by the layers to/from the GLViewWidget scene
        """
        if self.player:
            self.replay_controls.refresh()

        for layer in self.layers:
            added_graphics, removed_graphics = layer.updateGraphics()

            for added_graphic in added_graphics:
                self.graphics_view.gl_view_widget.addItem(added_graphic)

            for removed_graphic in removed_graphics:
                self.graphics_view.gl_view_widget.removeItem(removed_graphic)

    def setCameraView(self, camera_view):
        """Set the camera position to a preset camera view

        :param camera_view: the preset camera view

        """
        self.graphics_view.gl_view_widget.reset()
        if camera_view == CameraView.ORTHOGRAPHIC:
            self.graphics_view.gl_view_widget.setCameraPosition(
                pos=pg.Vector(0, 0, 0), distance=1000, elevation=90, azimuth=-90
            )
            self.graphics_view.gl_view_widget.setCameraParams(fov=1.0)
        elif camera_view == CameraView.LANDSCAPE_HIGH_ANGLE:
            self.graphics_view.gl_view_widget.setCameraPosition(
                pos=pg.Vector(0, -0.5, 0), distance=13, elevation=45, azimuth=-90
            )
        elif camera_view == CameraView.LEFT_HALF_HIGH_ANGLE:
            self.graphics_view.gl_view_widget.setCameraPosition(
                pos=pg.Vector(-2.5, 0, 0), distance=10, elevation=45, azimuth=180
            )
        elif camera_view == CameraView.RIGHT_HALF_HIGH_ANGLE:
            self.graphics_view.gl_view_widget.setCameraPosition(
                pos=pg.Vector(2.5, 0, 0), distance=10, elevation=45, azimuth=0
            )
