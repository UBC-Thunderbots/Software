import pyqtgraph as pg
from pyqtgraph.Qt.QtWidgets import *
import queue
import pyqtgraph.console as pg_console
from proto.robot_log_msg_pb2 import RobotLog, LogLevel
from proto.import_all_protos import *

import software.thunderscope.constants as constants
from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.thunderscope.log.g3log_checkboxes import g3logCheckboxes
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class g3logWidget(QWidget):
    def __init__(self, buffer_size=10):
        """The g3log widget is a console widget that displays g3log messages

        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        QWidget.__init__(self)

        self.console_widget = pg_console.ConsoleWidget()
        self.layout = QVBoxLayout()

        # disable input and buttons
        self.console_widget.input.hide()
        self.console_widget.ui.exceptionBtn.hide()
        self.console_widget.ui.historyBtn.hide()

        # Creates checkbox widget
        self.checkbox_widget = g3logCheckboxes()
        self.log_buffer = ThreadSafeBuffer(buffer_size, RobotLog,owner="G3LOG.log")

        self.layout.addWidget(self.console_widget)
        self.layout.addWidget(self.checkbox_widget)
        self.setLayout(self.layout)

        # LogLevel to string conversion map
        self.log_level_str_map = {
            LogLevel.DEBUG: "DEBUG",
            LogLevel.INFO: "INFO",
            LogLevel.WARNING: "WARNING",
            LogLevel.FATAL: "FATAL",
        }

    def refresh(self):
        """Update the log widget with another log message
        """
        # Need to make sure the message is new before logging it
        try:
            log = self.log_buffer.queue.get_nowait()
        except queue.Empty as empty:
            return

        # Checks whether this type of log is enabled from checkboxes
        if (
            (
                log.log_level == LogLevel.DEBUG
                and self.checkbox_widget.debug_checkbox.isChecked()
            )
            or (
                log.log_level == LogLevel.INFO
                and self.checkbox_widget.info_checkbox.isChecked()
            )
            or (
                log.log_level == LogLevel.WARNING
                and self.checkbox_widget.warning_checkbox.isChecked()
            )
            or (
                log.log_level == LogLevel.FATAL
                and self.checkbox_widget.fatal_checkbox.isChecked()
            )
        ):
            log_str = f"{log.created_timestamp.epoch_timestamp_seconds} {self.log_level_str_map[log.log_level]} [{log.file_name}->{log.line_number}] {log.log_msg}\n"
            self.console_widget.write(log_str)
        else:
            return
