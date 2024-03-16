import logging
import threading

from software.thunderscope.robot_diagnostics.controller_diagnostics import ControllerDiagnostics
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.robot_diagnostics.chicker_widget import ChickerWidget
from software.thunderscope.robot_diagnostics.diagnostics_input_widget import FullSystemConnectWidget, ControlMode
from software.thunderscope.robot_diagnostics.drive_and_dribbler_widget import DriveAndDribblerWidget
from pyqtgraph.Qt.QtCore import pyqtSignal
from pyqtgraph.Qt.QtWidgets import QWidget, QVBoxLayout


class DiagnosticsWidget(QWidget):
    # Signal to indicate if manual controls should be disabled based on boolean parameter
    diagnostics_input_mode_signal = pyqtSignal(bool)

    def __init__(self, proto_unix_io: ProtoUnixIO) -> None:
        super(DiagnosticsWidget, self).__init__()

        vbox_layout = QVBoxLayout()

        self.proto_unix_io = proto_unix_io

        self.diagnostics_control_input_widget = FullSystemConnectWidget(self.diagnostics_input_mode_signal)
        self.drive_dribbler_widget = DriveAndDribblerWidget(proto_unix_io)
        self.chicker_widget = ChickerWidget(proto_unix_io)
        self.controller = ControllerDiagnostics(proto_unix_io)


        self.__control_mode = ControlMode.DIAGNOSTICS

        self.diagnostics_control_input_widget.toggle_controls_signal.connect(
            lambda control_mode: self.toggle_control(control_mode)
        )

        self.run_diagnostics_thread = threading.Thread(
            target=self.__run_diagnostics_primivitve_set, daemon=True
        )

        vbox_layout.addWidget(self.diagnostics_control_input_widget)
        vbox_layout.addWidget(self.drive_dribbler_widget)
        vbox_layout.addWidget(self.chicker_widget)

        self.setLayout(vbox_layout)

    def refresh(self):
        if self.__control_mode == ControlMode.DIAGNOSTICS:
            # refresh the widgets so that they hold the most recent user control values
            self.diagnostics_control_input_widget.refresh()
            self.drive_dribbler_widget.refresh()
            self.chicker_widget.refresh()

            #
            diagnostics_primitive = DirectControlPrimitive(
                motor_control=self.drive_dribbler_widget.motor_control,
                power_control=self.chicker_widget.power_control,
            )

            # TODO: send the diagnostics primitive
            self.proto_unix_io.send_proto(DirectControlPrimitive, diagnostics_primitive)

        elif self.__control_mode == ControlMode.XBOX:
            # TODO: get values from controller widget
            logging.debug(self.controller.ang_vel)



    def toggle_control(self, mode: ControlMode):
        self.__control_mode = mode
        self.drive_dribbler_widget.set_enabled(mode == ControlMode.DIAGNOSTICS)
        self.controller.set_enabled(mode == ControlMode.XBOX)
