import pyqtgraph as pg
import pyqtgraph.console as pg_console
from software.networking.threaded_unix_listener import ThreadedUnixListener
import software.thunderscope.constants as constants
from software.thunderscope.log.g3log_checkboxes import g3logCheckboxes

from proto.robot_log_msg_pb2 import RobotLog, LogLevel


class g3logWidget(pg_console.ConsoleWidget):
    def __init__(self):
        pg_console.ConsoleWidget.__init__(self)

        self.log_receiver = ThreadedUnixListener(
            constants.UNIX_SOCKET_BASE_PATH + "log", RobotLog, convert_from_any=False
        )

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

        # Creates checkbox widget
        self.checkboxWidget = g3logCheckboxes()

    def refresh(self):
        """Update the log widget with another log message
        """
        log = self.log_receiver.maybe_pop()

        if not log:
            return

        # Checks whether this type of log is enabled from checkboxes
        if (
            (
                log.log_level == LogLevel.DEBUG
                and self.checkboxWidget.debug_checkbox.isChecked()
            )
            or (
                log.log_level == LogLevel.INFO
                and self.checkboxWidget.info_checkbox.isChecked()
            )
            or (
                log.log_level == LogLevel.WARNING
                and self.checkboxWidget.warning_checkbox.isChecked()
            )
            or (
                log.log_level == LogLevel.FATAL
                and self.checkboxWidget.fatal_checkbox.isChecked()
            )
        ):
            log_str = "{} {} [{}->{}] {}\n".format(
                log.created_timestamp.epoch_timestamp_seconds,
                log.log_level,
                log.file_name,
                log.line_number,
                log.log_msg,
            )
            self.write(log_str)
        else:
            return
