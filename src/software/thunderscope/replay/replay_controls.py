import pyqtgraph as pg
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.console as pg_console
from proto.robot_log_msg_pb2 import RobotLog, LogLevel
from proto.import_all_protos import *
from software.py_constants import *

import software.thunderscope.constants as constants
from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.thunderscope.log.g3log_checkboxes import g3logCheckboxes
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope import common_widgets


class ReplayControls(QWidget):
    def __init__(self, player):
        """The g3log widget is a console widget that displays g3log messages

        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary
        :param player: The player object

        """
        QWidget.__init__(self)

        self.layout = QHBoxLayout()
        self.player = player
        self.pressed = False

        if not self.player:
            return

        (
            self.replay_box,
            self.replay_slider,
            self.replay_label,
        ) = common_widgets.create_slider(
            "Replay", 0, self.player.end_time * MILLISECONDS_PER_SECOND, 1
        )

        self.layout.addWidget(self.replay_box)
        self.setLayout(self.layout)

        def __on_release():
            self.player.seek(self.replay_slider.value() / MILLISECONDS_PER_SECOND)
            self.player.play()
            self.pressed = False

        def __on_pressed():
            self.player.pause()
            self.pressed = True

        def __on_value_changed():
            if self.pressed:
                self.player.seek(self.replay_slider.value() / MILLISECONDS_PER_SECOND)

        self.replay_slider.valueChanged.connect(__on_value_changed)
        self.replay_slider.sliderReleased.connect(__on_release)
        self.replay_slider.sliderPressed.connect(__on_pressed)

    def refresh(self):
        if not self.pressed:
            self.replay_slider.setValue(
                self.player.current_packet_time * MILLISECONDS_PER_SECOND
            )
