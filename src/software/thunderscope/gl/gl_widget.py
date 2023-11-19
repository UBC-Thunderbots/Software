import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.opengl import *

import functools
import textwrap
import numpy as np

from software.thunderscope.constants import *

from software.thunderscope.gl.layers.gl_layer import GLLayer
from software.thunderscope.gl.layers.gl_measure_layer import GLMeasureLayer
from software.thunderscope.replay.proto_player import ProtoPlayer
from software.thunderscope.replay.replay_controls import ReplayControls
from software.thunderscope.gl.helpers.extended_gl_view_widget import *


class GLWidget(QWidget):
    """Widget that handles GLLayers to produce a 3D visualization of the field/world 
    and our AI. GLWidget can also provide replay controls.
    """

    def __init__(self, player: ProtoPlayer = None) -> None:
        """Initialize the GLWidget

        :param player: The replay player to optionally display media controls for

        """
        super().__init__()

        self.gl_view_widget = ExtendedGLViewWidget()
        self.setFocusPolicy(QtCore.Qt.FocusPolicy.StrongFocus)
        self.gl_view_widget.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)

        # Connect event handlers
        self.gl_view_widget.mouse_in_scene_pressed_signal.connect(
            self.mouse_in_scene_pressed
        )
        self.gl_view_widget.mouse_in_scene_dragged_signal.connect(
            self.mouse_in_scene_dragged
        )
        self.gl_view_widget.mouse_in_scene_released_signal.connect(
            self.mouse_in_scene_released
        )
        self.gl_view_widget.mouse_in_scene_moved_signal.connect(
            self.mouse_in_scene_moved
        )

        # Stylesheet for toolbar buttons
        tool_button_stylesheet = textwrap.dedent(
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

        # Setup Layers button for toggling visibility of layers
        self.layers_button = QPushButton()
        self.layers_button.setText("Layers")
        self.layers_button.setStyleSheet(tool_button_stylesheet)
        self.layers_menu = QMenu()
        self.layers_menu_actions = {}
        self.layers_button.setMenu(self.layers_menu)

        # Set up View button for setting the camera position to standard views
        self.camera_view_button = QPushButton()
        self.camera_view_button.setText("View")
        self.camera_view_button.setStyleSheet(tool_button_stylesheet)
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
        self.measure_button.setStyleSheet(tool_button_stylesheet)
        self.measure_button.setShortcut("m")
        self.measure_button.clicked.connect(lambda: self.toggle_measure_mode())

        # Setup Help button
        self.help_button = QPushButton()
        self.help_button.setText("Help")
        self.help_button.setStyleSheet(tool_button_stylesheet)
        self.help_button.clicked.connect(
            lambda: QMessageBox.information(self, "Help", THUNDERSCOPE_HELP_TEXT)
        )

        # Setup toolbar
        self.toolbar = QWidget()
        self.toolbar.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed
        )
        self.toolbar.setStyleSheet("background-color: black;" "padding: 0px;")
        self.toolbar.setLayout(QHBoxLayout())
        self.toolbar.layout().addWidget(self.layers_button)
        self.toolbar.layout().addStretch()
        self.toolbar.layout().addWidget(self.help_button)
        self.toolbar.layout().addWidget(self.measure_button)
        self.toolbar.layout().addWidget(self.camera_view_button)

        # Setup layout
        self.layout = QVBoxLayout()
        self.layout.setSpacing(0)
        self.layout.setContentsMargins(2, 2, 2, 2)
        self.setLayout(self.layout)
        self.layout.addWidget(self.toolbar)
        self.layout.addWidget(self.gl_view_widget)

        # Setup replay controls if player is provided and the log has some size
        self.player = player
        if self.player and self.player.end_time != 0.0:
            self.replay_controls = ReplayControls(player=player)
            self.replay_controls.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
            self.replay_controls.setSizePolicy(
                QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed
            )
            self.layout.addWidget(self.replay_controls)
        else:
            self.player = None

        self.layers = []

        self.set_camera_view(CameraView.LANDSCAPE_HIGH_ANGLE)

    def keyPressEvent(self, event: QtGui.QKeyEvent) -> None:
        """Detect when a key has been pressed
        
        :param event: The event
        
        """
        key_pressed = event.key()

        # Camera view shortcuts
        if key_pressed == Qt.Key.Key_1:
            self.set_camera_view(CameraView.ORTHOGRAPHIC)
        elif key_pressed == Qt.Key.Key_2:
            self.set_camera_view(CameraView.LANDSCAPE_HIGH_ANGLE)
        elif key_pressed == Qt.Key.Key_3:
            self.set_camera_view(CameraView.LEFT_HALF_HIGH_ANGLE)
        elif key_pressed == Qt.Key.Key_4:
            self.set_camera_view(CameraView.RIGHT_HALF_HIGH_ANGLE)

        # Propagate keypress event to all layers
        for layer in self.layers:
            layer.keyPressEvent(event)

    def keyReleaseEvent(self, event: QtGui.QKeyEvent) -> None:
        """Detect when a key has been released
        
        :param event: The event
        
        """
        # Propagate keypress event to all layers
        for layer in self.layers:
            layer.keyReleaseEvent(event)

    def mouse_in_scene_pressed(self, event: MouseInSceneEvent) -> None:
        """Propagate mouse_in_scene_pressed event to all layers
        
        :param event: The event
        
        """
        if self.measure_mode_enabled:
            # Only GLMeasureLayer should receive event to avoid layer
            # functionality conflicts
            self.measure_layer.mouse_in_scene_pressed(event)
        else:
            for layer in self.layers:
                layer.mouse_in_scene_pressed(event)

    def mouse_in_scene_dragged(self, event: MouseInSceneEvent) -> None:
        """Propagate mouse_in_scene_dragged event to all layers
        
        :param event: The event
        
        """
        if self.measure_mode_enabled:
            # Only GLMeasureLayer should receive event to avoid layer
            # functionality conflicts
            self.measure_layer.mouse_in_scene_dragged(event)
        else:
            for layer in self.layers:
                layer.mouse_in_scene_dragged(event)

    def mouse_in_scene_released(self, event: MouseInSceneEvent) -> None:
        """Propagate mouse_in_scene_released event to all layers
        
        :param event: The event
        
        """
        if self.measure_mode_enabled:
            # Only GLMeasureLayer should receive event to avoid layer
            # functionality conflicts
            self.measure_layer.mouse_in_scene_released(event)
        else:
            for layer in self.layers:
                layer.mouse_in_scene_released(event)

    def mouse_in_scene_moved(self, event: MouseInSceneEvent) -> None:
        """Propagate mouse_in_scene_moved event to all layers
        
        :param event: The event
        
        """
        if self.measure_mode_enabled:
            # Only GLMeasureLayer should receive event to avoid layer
            # functionality conflicts
            self.measure_layer.mouse_in_scene_moved(event)
        else:
            for layer in self.layers:
                layer.mouse_in_scene_moved(event)

    def add_layer(self, layer: GLLayer, visible: bool = True) -> None:
        """Add a layer to this GLWidget
        
        :param layer: The GLLayer 
        :param visible: Whether the layer is visible on startup

        """
        self.layers.append(layer)

        # Add the layer to the Layer menu
        # Not using a checkable QAction in order to prevent menu from closing
        # when an action is pressed
        layer_checkbox = QCheckBox(layer.name, self.layers_menu)
        layer_checkbox.setStyleSheet("QCheckBox { padding: 0px 8px; }")
        layer_checkbox.setChecked(visible)
        layer_action = QWidgetAction(self.layers_menu)
        layer_action.setDefaultWidget(layer_checkbox)
        self.layers_menu_actions[layer.name] = layer_action
        self.layers_menu.addAction(layer_action)

        # Add layer and its related layers to the scene
        while layer:
            self.gl_view_widget.addItem(layer)
            layer.setVisible(visible)

            # Connect visibility of all related layers to the same item
            # in the layer menu
            layer_checkbox.stateChanged.connect(
                functools.partial(
                    lambda l: l.setVisible(layer_checkbox.isChecked()), layer
                )
            )

            layer = layer.related_layer

    def remove_layer(self, layer: GLLayer) -> None:
        """Remove a layer from this GLWidget
        
        :param layer: The GLLayer to remove

        """
        self.layers.remove(layer)

        # Remove the layer from the Layer menu
        layer_action = self.layers_menu_actions[layer.name]
        self.layers_menu.removeAction(layer_action)

        # Remove layer its related layers from the scene
        while layer:
            self.gl_view_widget.removeItem(layer)
            layer = layer.related_layer

    def refresh(self) -> None:
        """Trigger an update on all the layers"""

        if self.player:
            self.replay_controls.refresh()

        # Prevents RuntimeError: wrapped C/C++ object of type ___ has been deleted
        # See: https://stackoverflow.com/a/60700622/20199855
        if self.isVisible() == False:
            return

        for layer in self.layers:
            while layer:
                if layer.visible():
                    layer.refresh_graphics()
                layer = layer.related_layer

    def set_camera_view(self, camera_view: CameraView) -> None:
        """Set the camera position to a preset camera view

        :param camera_view: the preset camera view

        """
        self.gl_view_widget.reset()
        if camera_view == CameraView.ORTHOGRAPHIC:
            self.gl_view_widget.setCameraPosition(
                pos=pg.Vector(0, 0, 0), distance=1100, elevation=90, azimuth=-90
            )
            self.gl_view_widget.setCameraParams(fov=1.0)
        elif camera_view == CameraView.LANDSCAPE_HIGH_ANGLE:
            self.gl_view_widget.setCameraPosition(
                pos=pg.Vector(0, -0.5, 0), distance=13, elevation=45, azimuth=-90
            )
        elif camera_view == CameraView.LEFT_HALF_HIGH_ANGLE:
            self.gl_view_widget.setCameraPosition(
                pos=pg.Vector(-2.5, 0, 0), distance=10, elevation=45, azimuth=180
            )
        elif camera_view == CameraView.RIGHT_HALF_HIGH_ANGLE:
            self.gl_view_widget.setCameraPosition(
                pos=pg.Vector(2.5, 0, 0), distance=10, elevation=45, azimuth=0
            )

    def toggle_measure_mode(self) -> None:
        """Toggles measure mode in the 3D visualizer"""

        self.measure_mode_enabled = not self.measure_mode_enabled

        # Enable/disable detect_mouse_movement_in_scene in ExtendedGLViewWidget
        # so that the mouse_in_scene_moved_signal is emitted if measure mode is on.
        #
        # Normally we want to disable detect_mouse_movement_in_scene so that we
        # don't do unnecessary calculations every tick to find the point in the scene
        # that the mouse is pointing at.
        self.gl_view_widget.detect_mouse_movement_in_scene = self.measure_mode_enabled

        if self.measure_mode_enabled:
            self.measure_layer = GLMeasureLayer("Measure")
            self.add_layer(self.measure_layer)
        else:
            self.remove_layer(self.measure_layer)
