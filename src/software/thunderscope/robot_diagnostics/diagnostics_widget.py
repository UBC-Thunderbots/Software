import logging

from pyqtgraph.Qt.QtCore import pyqtSignal
from pyqtgraph.Qt.QtWidgets import QWidget, QVBoxLayout
from proto.import_all_protos import *

from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.robot_diagnostics.chicker_widget import ChickerWidget
from software.thunderscope.robot_diagnostics.diagnostics_input_widget import (
    DiagnosticsInputModeWidget,
    ControlMode,
)
from software.thunderscope.robot_diagnostics.controller_diagnostics import (
    ControllerInputHandler,
)
from software.thunderscope.robot_diagnostics.drive_and_dribbler_widget import (
    DriveAndDribblerWidget,
)


class DiagnosticsWidget(QWidget):
    # Signal to indicate if manual controls should be disabled based on boolean parameter
    diagnostics_input_mode_signal = pyqtSignal(bool)

    def __init__(self, proto_unix_io: ProtoUnixIO) -> None:
        super(DiagnosticsWidget, self).__init__()

        vbox_layout = QVBoxLayout()

        self.proto_unix_io = proto_unix_io

        self.controller_handler = ControllerInputHandler(proto_unix_io)
        self.drive_dribbler_widget = DriveAndDribblerWidget(proto_unix_io)
        self.chicker_widget = ChickerWidget(proto_unix_io)
        self.diagnostics_control_input_widget = DiagnosticsInputModeWidget(
            self.diagnostics_input_mode_signal
        )

        self.__control_mode = ControlMode.DIAGNOSTICS

        self.diagnostics_control_input_widget.toggle_controls_signal.connect(
            lambda control_mode: self.toggle_control(control_mode)
        )

        # self.run_diagnostics_thread = threading.Thread(
        #     target=self.__run_diagnostics_primivitve_set, daemon=True
        # )

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

            # TODO: throws error on exit
            diagnostics_primitive = Primitive(
                direct_control=DirectControlPrimitive(
                    motor_control=self.drive_dribbler_widget.motor_control,
                    power_control=self.chicker_widget.power_control,
                )
            )

            self.proto_unix_io.send_proto(Primitive, diagnostics_primitive)

        elif self.__control_mode == ControlMode.XBOX:
            # TODO: get values from controller widget, placeholder log for now
            if self.controller_handler.controller is not None:
                logging.debug(self.controller_handler.ang_vel)

    def toggle_control(self, mode: ControlMode):
        self.__control_mode = mode
        self.drive_dribbler_widget.set_enabled(mode == ControlMode.DIAGNOSTICS)
        self.controller_handler.set_enabled(mode == ControlMode.XBOX)
