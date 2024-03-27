from software.thunderscope.robot_diagnostics.controller_view import ControllerStatusView


class ControllerConnectionHandler(QWidget):

    def __enter__(self) -> None:
        super(ControllerConnectionHandler, self).__init__()

        self.controller_view = ControllerStatusView()