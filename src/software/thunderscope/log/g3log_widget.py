import pyqtgraph as pg
import pyqtgraph.console as pg_console
from software.networking.threaded_unix_listener import ThreadedUnixListener
from proto.robot_log_msg_pb2 import RobotLog


class g3logWidget(pg_console.ConsoleWidget):
    def __init__(self):
        pg_console.ConsoleWidget.__init__(self)

        self.log_receiver = ThreadedUnixListener(
            "/tmp/tbots/log", RobotLog, convert_from_any=False
        )

        # disable input and buttons
        self.input.hide()
        self.ui.exceptionBtn.hide()
        self.ui.historyBtn.hide()

        # TODO remove - dark theme hacks
        self.ui.output.setStyleSheet(
            """QPlainTextEdit{
                color: #ffffff;
                background-color: #000000;
                selection-background-color: #606060;
                selection-color: #ffffff;
            }"""
        )

    def refresh(self):
        log = self.log_receiver.maybe_pop()

        if not log:
            return

        log_str = "{} {} [{}->{}] {}\n".format(
            log.created_timestamp,
            log.log_level,
            log.file_name,
            log.line_number,
            log.log_msg,
        )

        self.write(log_str)
