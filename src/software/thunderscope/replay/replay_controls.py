import pyqtgraph as pg
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt import QtCore, QtGui
import queue
import pyqtgraph.console as pg_console
from proto.robot_log_msg_pb2 import RobotLog, LogLevel
from proto.import_all_protos import *

import software.thunderscope.constants as constants
from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.thunderscope.log.g3log_checkboxes import g3logCheckboxes
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope import common_widgets


class ReplayControls(QWidget):
    def __init__(self):
        """The g3log widget is a console widget that displays g3log messages

        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        QWidget.__init__(self)

        self.layout = QHBoxLayout()

        self.replay_box, self.replay_slider, self.replay_label =\
                    common_widgets.create_slider("Replay", 0, 100, 0.01)

        self.layout.addWidget(self.replay_box)
        self.setLayout(self.layout)

    def play(self):
        print("Play")

    def refresh(self):
        pass
