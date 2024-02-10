from software.thunderscope.robot_diagnostics.controller_diagnostics import ControllerDiagnostics
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.robot_diagnostics.chicker_widget import ChickerWidget
from software.thunderscope.robot_diagnostics.diagnostics_input_widget import FullSystemConnectWidget, ControlMode
from software.thunderscope.robot_diagnostics.drive_and_dribbler_widget import DriveAndDribblerWidget


class DiagnosticsWidget(TScopeWidget):

    # Signal to indicate if manual controls should be disabled based on boolean parameter
    # diagnostics_input_mode_signal = pyqtSignal(bool)
    def __init__(self, proto_unix_io: ProtoUnixIO) -> None:
        super(DiagnosticsWidget, self).__init__()

        vbox_layout = QVBoxLayout()

        self.diagnostics_control_input_widget = FullSystemConnectWidget(self.diagnostics_input_mode_signal)
        self.drive_dribbler_widget = DriveAndDribblerWidget(proto_unix_io)
        self.chicker_widget = ChickerWidget(proto_unix_io)
        self.controller = ControllerDiagnostics(proto_unix_io)

        self.diagnostics_control_input_widget.toggle_control_signal.connect(
            lambda control_mode: self.controller.toggle_input_mode(
                control_mode == ControlMode.DIAGNOSTICS
            )
        )

        vbox_layout.addWidget(self.diagnostics_control_input_widget)
        vbox_layout.addWidget(self.drive_dribbler_widget)
        vbox_layout.addWidget(self.chicker_widget)

        self.setLayout(vbox_layout)

    def update_mode(self, mode: ControlMode):
        self.drive_dribbler_widget.set_enabled(mode == ControlMode.DIAGNOSTICS)
        self.controller.set_enabled(mode == ControlMode.XBOX)
