import time
import pyqtgraph as pg
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt import QtCore, QtGui
from functools import partial

from software.thunderscope.field.field_layer import FieldLayer
from software.thunderscope import common_widgets
from software.py_constants import *


class ReplayControls(QGroupBox):
    def __init__(self, player):
        """Setup the replay controls. 

        :param player: The player to control.

        """
        QGroupBox.__init__(self)

        self.setFocusPolicy(QtCore.Qt.FocusPolicy.StrongFocus)
        self.setFocus()

        self.controls_layout = QVBoxLayout()
        self.player = player

        self.buttons = QGroupBox()
        self.buttons_layout = QHBoxLayout()

        # Set up play button
        self.play_pause = QPushButton()
        self.play_pause.setText("⏸")
        self.play_pause.clicked.connect(self.__on_play_pause_clicked)

        # Set up back buttons
        self.restart = QPushButton()
        self.restart.setText("⏮")
        self.restart.clicked.connect(partial(self.seek_absolute, 0))

        self.back1min = QPushButton()
        self.back1min.setText("↶\n1 min")
        self.back1min.clicked.connect(partial(self.seek_relative, -60))

        self.back10s = QPushButton()
        self.back10s.setText("↶\n10 s")
        self.back10s.clicked.connect(partial(self.seek_relative, -10))

        self.back1entry = QPushButton()
        self.back1entry.setText("↶\n1 frame")
        self.back1entry.clicked.connect(self.player.single_step_backward)

        # Set up forward buttons
        self.forward1min = QPushButton()
        self.forward1min.setText("↷\n1 min")
        self.forward1min.clicked.connect(partial(self.seek_relative, 60))

        self.forward10s = QPushButton()
        self.forward10s.setText("↷\n10 s")
        self.forward10s.clicked.connect(partial(self.seek_relative, 10))

        self.forward1entry = QPushButton()
        self.forward1entry.setText("↷\n1 frame")
        self.forward1entry.clicked.connect(self.player.single_step_forward)

        # Setup playback speed combo box
        self.playback_speed_combo_box = QtGui.QComboBox(self)

        for item in ["3", "2", "1", "0.5", "0.2", "0.1", "0.05", "0.01"]:
            self.playback_speed_combo_box.addItem(item)

        # Default to 1x which is at index 2
        self.playback_speed_combo_box.setCurrentIndex(2)
        self.playback_speed_combo_box.currentTextChanged.connect(
            self.player.set_playback_speed
        )

        self.buttons_layout.addWidget(self.restart)
        self.buttons_layout.addWidget(self.back1min)
        self.buttons_layout.addWidget(self.back10s)
        self.buttons_layout.addWidget(self.back1entry)
        self.buttons_layout.addWidget(self.play_pause)
        self.buttons_layout.addWidget(self.playback_speed_combo_box)
        self.buttons_layout.addWidget(self.forward1entry)
        self.buttons_layout.addWidget(self.forward10s)
        self.buttons_layout.addWidget(self.forward1min)
        self.buttons.setLayout(self.buttons_layout)

        self.slider_pressed = False
        # tracks history of the state of is_playing before a button is pressed
        self.was_playing = False

        # Setup the replay slider
        (
            self.replay_box,
            self.replay_slider,
            self.replay_label,
        ) = common_widgets.create_slider(
            "Replay",
            min_val=0,
            max_val=self.player.end_time * MILLISECONDS_PER_SECOND,
            tick_spacing=1,
        )

        # Setup mouse interactions
        self.replay_slider.valueChanged.connect(self.__on_replay_slider_value_changed)
        self.replay_slider.sliderReleased.connect(self.__on_replay_slider_released)
        self.replay_slider.sliderPressed.connect(self.__on_replay_slider_pressed)

        self.controls_layout.addWidget(self.replay_box)
        self.controls_layout.addWidget(self.buttons)
        self.setLayout(self.controls_layout)

    def __on_play_pause_clicked(self):
        """When the play/pause button is clicked, toggle play/pause and set the text
        """
        self.player.toggle_play_pause()

    def __on_replay_slider_released(self):
        """When the slider is released, seek to the sliders location and
        start playing.

        """
        self.player.seek(self.replay_slider.value() / MILLISECONDS_PER_SECOND)
        if self.was_playing:
            self.player.play()
        self.slider_pressed = False

    def __on_replay_slider_pressed(self):
        """When the slider is pressed, pause the player so the slider
        doesn't move away.

        """
        self.was_playing = self.player.is_playing
        self.player.pause()
        self.slider_pressed = True

    def __on_replay_slider_value_changed(self, event):
        """When the slider value is changed, update the label to show the
        current time.

        """
        current_time = time.strftime(
            "%H:%M:%S",
            time.gmtime(self.replay_slider.value() / MILLISECONDS_PER_SECOND),
        )
        self.replay_label.setText("Current time: {}".format(current_time))

    def refresh(self):
        """Refresh the slider to match the current time.

        """
        if not self.slider_pressed:
            self.replay_slider.setValue(
                self.player.current_packet_time * MILLISECONDS_PER_SECOND
            )

        self.play_pause.setText("⏸" if self.player.is_playing else "▶")

    def keyPressEvent(self, event):
        """When a key is pressed, pause the player.
        
        """
        if event.key() == QtCore.Qt.Key.Key_P:
            self.__on_play_pause_clicked()

    def seek_relative(self, relative_time):
        """Seeks time relative to current time

        :param relative_time The time relative to the current time to seek to

        """
        self.was_playing = self.player.is_playing
        self.player.pause()
        self.player.seek(self.player.current_packet_time + relative_time)
        if self.was_playing:
            self.player.play()

    def seek_absolute(self, absolute_time):
        self.was_playing = self.player.is_playing
        self.player.pause()
        self.player.seek(absolute_time)
        if self.was_playing:
            self.player.play()
