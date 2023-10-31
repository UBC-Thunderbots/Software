import pyqtgraph as pg
from pyqtgraph.Qt.QtWidgets import *
import queue
from software.py_constants import *
import pyqtgraph.console as pg_console
from proto.robot_log_msg_pb2 import RobotLog, LogLevel
from proto.import_all_protos import *

import software.thunderscope.constants as constants
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
        self.console_widget.setStyleSheet(
            """
            border: none;
            border-radius: 5px;
            background: #232629;
            """
        )

        self.layout = QVBoxLayout()

        # disable input and buttons
        self.console_widget.input.hide()
        self.console_widget.exceptionBtn.hide()
        self.console_widget.historyBtn.hide()

        # _lastCommandRow is initialized to None, which causes errors
        # when writing to ReplWidget
        self.console_widget.repl._lastCommandRow = 0

        # Creates checkbox widget
        self.checkbox_widget = g3logCheckboxes()
        self.log_buffer = ThreadSafeBuffer(buffer_size, RobotLog)

        self.layout.addWidget(self.console_widget)
        self.layout.addWidget(self.checkbox_widget)
        self.setLayout(self.layout)

        # ignore repeated crash proto
        self.robot_last_fatal_time_s = []
        for id in range(MAX_ROBOT_IDS_PER_SIDE):
            self.robot_last_fatal_time_s.append(0)

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
                log.log_level == LogLevel.CONTRACT
                and self.checkbox_widget.fatal_checkbox.isChecked()
            )
            or (
                log.log_level == LogLevel.FATAL
                and self.checkbox_widget.fatal_checkbox.isChecked()
            )
        ):
            log_str = f"R{log.robot_id} {log.created_timestamp.epoch_timestamp_seconds} {constants.LOG_LEVEL_STR_MAP[log.log_level]} [{log.file_name}->{log.line_number}] {log.log_msg}\n"
            self.console_widget.repl.write(log_str, style="output")
        else:
            return
