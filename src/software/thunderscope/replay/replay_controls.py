import time
import pyqtgraph as pg
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt import QtCore, QtGui
from functools import partial

from software.thunderscope.common import common_widgets
from software.py_constants import *


class ReplayControls(QWidget):
    def __init__(self, player):
        """Setup the replay controls. 

        :param player: The player to control.

        """
        QGroupBox.__init__(self)

        self.setFocusPolicy(QtCore.Qt.FocusPolicy.StrongFocus)
        self.setFocus()

        self.player = player

        self.controls_layout = QVBoxLayout()
        self.controls_layout.setContentsMargins(12, 12, 12, 12)
        self.buttons_layout = QHBoxLayout()

        for button in [
            ("⏮", partial(self.seek_absolute, 0)),
            ("↶\n1 min", partial(self.seek_relative, -60)),
            ("↶\n10 s", partial(self.seek_relative, -10)),
            ("↶\n1 s", partial(self.seek_relative, -1)),
        ]:
            qbutton = QPushButton()
            qbutton.setText(button[0])
            qbutton.clicked.connect(button[1])
            self.buttons_layout.addWidget(qbutton)

        # Set up play button
        self.play_pause = QPushButton()
        self.play_pause.setText("⏸")
        self.play_pause.clicked.connect(self.__on_play_pause_clicked)
        self.buttons_layout.addWidget(self.play_pause)

        # Setup playback speed combo box
        self.playback_speed_combo_box = QtGui.QComboBox(self)

        for item in ["3", "2", "1", "0.5", "0.2", "0.1", "0.05", "0.01"]:
            self.playback_speed_combo_box.addItem(item)

        # Default to 1x which is at index 2
        self.playback_speed_combo_box.setCurrentIndex(2)
        self.playback_speed_combo_box.currentTextChanged.connect(
            self.player.set_playback_speed
        )
        self.buttons_layout.addWidget(self.playback_speed_combo_box)

        for button in [
            ("↷\nStep", self.player.single_step_forward),
            ("↷\n1 s", partial(self.seek_relative, 1)),
            ("↷\n10 s", partial(self.seek_relative, 10)),
            ("↷\n1 min", partial(self.seek_relative, 60)),
            ("⏭", partial(self.seek_absolute, self.player.end_time)),
        ]:
            qbutton = QPushButton()
            qbutton.setText(button[0])
            qbutton.clicked.connect(button[1])
            self.buttons_layout.addWidget(qbutton)

        # Set up save clip button
        self.save_clip = QPushButton()
        self.save_clip.setText("Start\nClip")
        self.save_clip.clicked.connect(self.__on_save_clip_clicked)
        self.buttons_layout.addWidget(self.save_clip)

        self.slider_pressed = False
        # tracks history of the state of is_playing before a button is pressed
        self.was_playing = False
        self.clipping = False
        self.clip_start = 0

        # Setup the replay slider
        (
            self.replay_layout,
            self.replay_slider,
            self.replay_label,
        ) = common_widgets.create_slider(
            text="",
            min_val=0,
            max_val=self.player.end_time * MILLISECONDS_PER_SECOND,
            tick_spacing=1,
        )

        # Setup mouse interactions
        self.replay_slider.valueChanged.connect(self.__on_replay_slider_value_changed)
        self.replay_slider.sliderReleased.connect(self.__on_replay_slider_released)
        self.replay_slider.sliderPressed.connect(self.__on_replay_slider_pressed)

        self.controls_layout.addLayout(self.replay_layout)
        self.controls_layout.addLayout(self.buttons_layout)
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
        total_time = time.strftime(
            "%H:%M:%S",
            time.gmtime(self.replay_slider.maximum() / MILLISECONDS_PER_SECOND),
        )
        self.replay_label.setText(f"{current_time} / {total_time}")

    def refresh(self):
        """Refresh the slider to match the current time.

        """
        if not self.slider_pressed:
            self.replay_slider.setValue(
                self.player.current_packet_time * MILLISECONDS_PER_SECOND
            )

        self.play_pause.setText("⏸" if self.player.is_playing else "▶")
        self.save_clip.setText(
            "Save\nClip"
            if self.clipping and self.player.current_packet_time > self.clip_start
            else "Start\nClip"
        )

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
        """Seeks to an absolute time

        :param absolute_time The absolute time to seek to

        """
        self.was_playing = self.player.is_playing
        self.player.pause()
        self.player.seek(absolute_time)
        if self.was_playing:
            self.player.play()

    def __on_save_clip_clicked(self):
        """When the button is clicked, save clip if current time is after the clip start time
        """
        if self.clipping and self.player.current_packet_time > self.clip_start:
            self.player.pause()
            end_time = self.player.current_packet_time
            filename, _ = QtGui.QFileDialog.getSaveFileName(
                self,
                "Save clip",
                "~/log_clip_{}.replay".format(int(time.time())),
                options=QFileDialog.Option.DontUseNativeDialog,
            )
            self.player.save_clip(filename, self.clip_start, end_time)
            self.clipping = False
        else:
            self.clipping = True
            self.was_playing = self.player.is_playing
            self.player.pause()
            self.clip_start = self.player.current_packet_time
            if self.was_playing:
                self.player.play()
