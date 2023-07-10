import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.opengl import *

from software.thunderscope.constants import CameraView

from software.thunderscope.gl.gl_layer import GLLayer
from software.thunderscope.replay.replay_controls import ReplayControls

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

        self.setFocusPolicy(QtCore.Qt.FocusPolicy.StrongFocus)
        self.graphics_view.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)

        # Setup legend
        self.legend = pg.LegendItem((80, 60))
        self.graphics_view.addItem(self.legend)

        # Set up menu button for setting the camera position to standard views
        self.camera_view_actions = [
            QtGui.QAction("Landscape Top Down"),
            QtGui.QAction("Landscape High Angle"),
            QtGui.QAction("Left Half High Angle"),
            QtGui.QAction("Right Half High Angle"),
        ]
        self.camera_view_actions[0].triggered.connect(
            lambda: self.setCameraView(CameraView.LANDSCAPE_TOP_DOWN)
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

        self.camera_view_button = QToolButton()
        self.camera_view_button.setText("View ")
        self.camera_view_button.setStyleSheet(
            "QToolButton {"
            "   background-color: transparent;" 
            "   border-color: transparent;"
            "   border-width: 0px;"
            "   height: 8px;"
            "   padding-right: 8px;"
            "}"
        )

        self.camera_view_menu = QtGui.QMenu()
        self.camera_view_button.setMenu(self.camera_view_menu)
        self.camera_view_button.setPopupMode(QToolButton.ToolButtonPopupMode.InstantPopup)
        
        for camera_view_action in self.camera_view_actions:
            self.camera_view_menu.addAction(camera_view_action)
        
        self.toolbar = QWidget()
        self.toolbar.setMaximumHeight(40)
        self.toolbar.setStyleSheet(
            "background-color: black;"
            "padding: 0px;"
        )
        self.toolbar.setLayout(QHBoxLayout())
        self.toolbar.layout().addStretch()
        self.toolbar.layout().addWidget(self.camera_view_button)

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

        self.layers = []

        self.setCameraView(CameraView.LANDSCAPE_HIGH_ANGLE)

    def keyPressEvent(self, event):
        """Propagate keypress event to all field layers
        
        :param event: The event
        
        """
        for layer in self.layers:
            layer.keyPressEvent(event)

    def keyReleaseEvent(self, event):
        """Propagate keyrelease event to all field layers
        
        :param event: The event
        
        """
        for layer in self.layers:
            layer.keyReleaseEvent(event)

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
        """Triggers an update on all the layers, adding/removing GLGraphicsItem 
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
        if camera_view == CameraView.LANDSCAPE_TOP_DOWN:
            self.graphics_view.gl_view_widget.setCameraPosition(
                pos=pg.Vector(0, 0, 0),
                distance=15,
                elevation=90,
                azimuth=-90
            )
        elif camera_view == CameraView.LANDSCAPE_HIGH_ANGLE:
            self.graphics_view.gl_view_widget.setCameraPosition(
                pos=pg.Vector(0, -0.5, 0),
                distance=13,
                elevation=45,
                azimuth=-90
            )
        elif camera_view == CameraView.LEFT_HALF_HIGH_ANGLE:
            self.graphics_view.gl_view_widget.setCameraPosition(
                pos=pg.Vector(-2.5, 0, 0),
                distance=10,
                elevation=45,
                azimuth=180
            )
        elif camera_view == CameraView.RIGHT_HALF_HIGH_ANGLE:
            self.graphics_view.gl_view_widget.setCameraPosition(
                pos=pg.Vector(2.5, 0, 0),
                distance=10,
                elevation=45,
                azimuth=0
            )

class GraphicsView(pg.GraphicsView):
    """GraphicsView subclass that uses GLViewWidget as its canvas. 
    This allows 2D graphics to be overlaid on a 3D background.
    """

    def __init__(self):
        self.gl_view_widget = GLViewWidget()
        pg.GraphicsView.__init__(self, background=None)
        self.setStyleSheet("background: transparent")
        self.setViewport(self.gl_view_widget)
        
    def paintEvent(self, event):
        """Propagate paint events to both widgets

        :param event: The event

        """
        self.gl_view_widget.paintEvent(event)
        return pg.GraphicsView.paintEvent(self, event)

    def mousePressEvent(self, event):
        """Propagate mouse events to both widgets
        
        :param event: The event
        
        """
        pg.GraphicsView.mousePressEvent(self, event)
        self.gl_view_widget.mousePressEvent(event)
        
    def mouseMoveEvent(self, event):
        """Propagate mouse events to both widgets
        
        :param event: The event
        
        """
        pg.GraphicsView.mouseMoveEvent(self, event)
        self.gl_view_widget.mouseMoveEvent(event)
        
    def mouseReleaseEvent(self, event):
        """Propagate mouse events to both widgets
        
        :param event: The event
        
        """
        pg.GraphicsView.mouseReleaseEvent(self, event)
        self.gl_view_widget.mouseReleaseEvent(event)

    def wheelEvent(self, event):
        """Propagate mouse wheel events to both widgets
        
        :param event: The event
        
        """
        pg.GraphicsView.wheelEvent(self, event)
        self.gl_view_widget.wheelEvent(event)
