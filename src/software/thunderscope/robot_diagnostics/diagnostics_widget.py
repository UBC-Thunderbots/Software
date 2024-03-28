import logging
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt.QtCore import *
from proto.import_all_protos import *

from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.robot_diagnostics.chicker_widget import ChickerWidget
from software.thunderscope.robot_diagnostics.controller_view import ControllerStatusView
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

        self.proto_unix_io = proto_unix_io

        self.controller_handler = ControllerInputHandler(proto_unix_io)
        self.drive_dribbler_widget = DriveAndDribblerWidget(proto_unix_io)
        self.chicker_widget = ChickerWidget(proto_unix_io)
        self.diagnostics_control_input_widget = DiagnosticsInputModeWidget(
            self.diagnostics_input_mode_signal,
        )

        self.__control_mode = ControlMode.DIAGNOSTICS

        self.diagnostics_control_input_widget.toggle_controls_signal.connect(
            lambda control_mode: self.toggle_control(control_mode)
        )

        # Inlined Controller Status and Refresh Button Widget
        # Comment for PR:
        # Easier to have it at this widget level for
        # communication and logic handling to other widgets

        self.controller_refresh_button = QPushButton()
        self.controller_refresh_button.setText("Re-initialize Handheld Controller")
        self.controller_refresh_button.clicked.connect(self.__refresh_controller)

        self.controller_status = ControllerStatusView()

        vbox_layout = QVBoxLayout()

        vbox_layout.addWidget(self.controller_refresh_button)
        vbox_layout.addWidget(self.controller_status)
        vbox_layout.addWidget(self.diagnostics_control_input_widget)
        vbox_layout.addWidget(self.drive_dribbler_widget)
        vbox_layout.addWidget(self.chicker_widget)

        self.setLayout(vbox_layout)

    def toggle_control(self, mode: ControlMode):
        self.__control_mode = mode
        self.drive_dribbler_widget.set_enabled(mode == ControlMode.DIAGNOSTICS)
        self.controller_handler.set_controller_enabled(mode == ControlMode.XBOX)

    def __refresh_controller(self) -> None:
        logging.debug("Attempting to reinitialize handheld controller...")
        if not self.controller_handler.controller_initialized():
            self.controller_handler = ControllerInputHandler(self.proto_unix_io)
            if self.controller_handler.controller_initialized():
                logging.debug("Successfully reinitialized handheld controller")
                self.controller_status.set_connected()
            else:
                logging.debug("Failed to reinitialize handheld controller")
                self.controller_status.set_disconnected()
        else:
            logging.debug("Handheld controller is initialized and connected...")

    def refresh(self):
        # check if controller is initialized and set status
        if self.controller_handler.controller_initialized():
            self.controller_status.set_connected()
        else:
            self.controller_status.set_disconnected()

        if self.__control_mode == ControlMode.DIAGNOSTICS:
            # get the values from GUI sliders and send values to proto_unix_io
            self.diagnostics_control_input_widget.refresh()
            self.drive_dribbler_widget.refresh()
            self.chicker_widget.refresh()

            diagnostics_primitive = Primitive(
                direct_control=DirectControlPrimitive(
                    motor_control=self.drive_dribbler_widget.motor_control,
                    power_control=self.chicker_widget.power_control,
                )
            )

            self.proto_unix_io.send_proto(Primitive, diagnostics_primitive)

        elif self.__control_mode == ControlMode.XBOX:
            # get the values from handheld controller and send values to proto_unix_io
            diagnostics_primitive = (
                self.controller_handler.get_latest_primitive_command()
            )

            if diagnostics_primitive is not None:
                logging.debug("Sending diagnostics primitive: " + diagnostics_primitive)
                self.proto_unix_io.send_proto(Primitive, diagnostics_primitive)
            else:
                logging.debug("Sending stop primitive")
                self.proto_unix_io.send_proto(Primitive, StopPrimitive())

    # TODO: ensure this is actually called and closed properly
    def close(self):
        self.controller_handler.close()
