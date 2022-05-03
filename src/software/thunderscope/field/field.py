import time
import pyqtgraph as pg
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt import QtCore, QtGui

from software.thunderscope.field.field_layer import FieldLayer
from software.thunderscope import common_widgets
from software.py_constants import *


class Field(QWidget):

    """Wrapper to handle Field Layers"""

    def __init__(self, player=None, max_x_range=10000, max_y_range=6000):
        """Initialize the field

        :param player: The replay player to optionally display media controls for
        :param max_x_range: Maximum x range of the field
        :param max_y_range: Maximum y range of the field

        """
        QWidget.__init__(self)
        self.plot_widget = pg.PlotWidget()

        # Setup Field Plot
        self.plot_widget.setAspectLocked()
        self.plot_widget.showGrid(x=True, y=True, alpha=0.5)

        self.max_x_range = max_x_range
        self.max_y_range = max_y_range

        # NOTE: This line has caused a lot of grief. DO NOT remove this, or you
        # will encounter a severe performance hit.
        #
        # If auto range is enabled, pyqtgraph will recalculate the range of the
        # field plot every time something changes. Since we have a lot of things
        # going on the field, this really bogs down the system.
        #
        # At the time of writing, this capped us at 30 FPS (when we were
        # trying to run at 60fps). Removing this allowed us to run at 60 FPS.
        #
        # More info here: https://groups.google.com/g/pyqtgraph/c/7dR_-3EP29k
        self.plot_widget.disableAutoRange()

        # Only call auto range once
        self.range_set = False

        self.plot_widget.setMouseTracking(True)

        # Setup Field Plot Legend
        self.legend = pg.LegendItem((80, 60), offset=(70, 20))
        self.legend.setParentItem(self.plot_widget.graphicsItem())

        # Fields
        self.layers = []

        self.layout = QVBoxLayout()
        self.layout.addWidget(self.plot_widget)
        self.media_layout = QHBoxLayout()
        self.player = player
        self.pressed = False

        self.group_box = QGroupBox()

        self.play = QPushButton()
        self.play.setEnabled(True)
        self.play.setText("Play")
        self.play.clicked.connect(self.player.play)

        self.comboBox = QtGui.QComboBox(self)
        self.comboBox.addItem("3")
        self.comboBox.addItem("2")
        self.comboBox.addItem("1")
        self.comboBox.addItem("0.5")
        self.comboBox.addItem("0.2")
        self.comboBox.addItem("0.1")
        self.comboBox.setCurrentIndex(2)

        self.comboBox.currentTextChanged.connect(self.player.set_playback_speed)

        self.pause = QPushButton()
        self.pause.setEnabled(True)
        self.pause.setText("Pause")
        self.pause.clicked.connect(self.player.pause)

        self.media_layout.addWidget(self.play)
        self.media_layout.addWidget(self.pause)
        self.media_layout.addWidget(self.comboBox)
        self.replay_slider = None

        def __on_release():
            self.player.seek(self.replay_slider.value() / MILLISECONDS_PER_SECOND)
            self.player.play()
            self.pressed = False

        def __on_pressed():
            self.player.pause()
            self.pressed = True

        def __on_value_changed():
            current_time = time.strftime(
                "%H:%M:%S",
                time.gmtime(self.replay_slider.value() / MILLISECONDS_PER_SECOND),
            )
            self.replay_label.setText("Current time: {}".format(current_time))

        # If there is a player and the loaded log has some data
        if self.player is not None and self.player.end_time != 0:
            (
                self.replay_box,
                self.replay_slider,
                self.replay_label,
            ) = common_widgets.create_slider(
                "Replay", 0, self.player.end_time * MILLISECONDS_PER_SECOND, 1
            )
            self.replay_slider.valueChanged.connect(__on_value_changed)
            self.replay_slider.sliderReleased.connect(__on_release)
            self.replay_slider.sliderPressed.connect(__on_pressed)
            self.media_layout.addWidget(self.replay_box)

        self.group_box.setLayout(self.media_layout)
        self.layout.addWidget(self.group_box)
        self.setLayout(self.layout)

    def keyPressEvent(self, event):
        """Propagate keypress event to all field layers
        
        :param event: The event
        
        """
        self.plot_widget.setMouseEnabled(x=False, y=False)
        for layer in self.layers:
            layer.keyPressEvent(event)

    def keyReleaseEvent(self, event):
        """Propagate keyrelease event to all field layers
        
        :param event: The event
        
        """
        self.plot_widget.setMouseEnabled(x=True, y=True)
        for layer in self.layers:
            layer.keyReleaseEvent(event)

    def add_layer(self, name: str, layer: FieldLayer):
        """Add a layer to this field and to the legend.
        
        :param name: The name of the layer
        :param layer: The FieldLayer graphics object

        """
        self.layers.append(layer)
        self.plot_widget.addItem(layer)
        self.legend.addItem(layer, name)

    def refresh(self):
        """Trigger an update on all the layers
        """
        if not self.pressed and self.replay_slider:
            self.replay_slider.setValue(
                self.player.current_packet_time * MILLISECONDS_PER_SECOND
            )

        for layer in self.layers:
            layer.update()

        # Set the field range once
        if not self.range_set:
            self.plot_widget.setRange(
                xRange=(-int(self.max_x_range / 2), int(self.max_x_range / 2)),
                yRange=(-int(self.max_y_range / 2), int(self.max_y_range / 2)),
            )
            self.range_set = True
