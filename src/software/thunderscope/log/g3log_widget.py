import pyqtgraph as pg
import pyqtgraph.console as pg_console
from software.networking.threaded_unix_listener import ThreadedUnixListener
import software.thunderscope.constants as constants

from proto.robot_log_msg_pb2 import RobotLog
import queue


class g3logWidget(pg_console.ConsoleWidget):
    def __init__(self, buffer_size=10):
        pg_console.ConsoleWidget.__init__(self)

        # disable input and buttons
        self.input.hide()
        self.ui.exceptionBtn.hide()
        self.ui.historyBtn.hide()

        # flip the text and background color to make it dark theme
        self.ui.output.setStyleSheet(
            """QPlainTextEdit{
                color: #ffffff;
                background-color: #000000;
                selection-background-color: #606060;
                selection-color: #ffffff;
            }"""
        )

        self.log_buffer = queue.Queue(buffer_size)

    def refresh(self):
        """Update the log widget with another log message
        """
        try:
            log = self.log_buffer.get_nowait()
        except queue.Empty as empty:
            return

        log_str = "{} {} [{}->{}] {}\n".format(
            log.created_timestamp.epoch_timestamp_seconds,
            log.log_level,
            log.file_name,
            log.line_number,
            log.log_msg,
        )

        self.write(log_str)
