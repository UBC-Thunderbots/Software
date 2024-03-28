import logging
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt.QtCore import *
from proto.import_all_protos import *

from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.robot_diagnostics.chicker_widget import ChickerWidget
from software.thunderscope.robot_diagnostics.controller_status_view import (
    ControllerStatusView,
    ControllerConnected,
)
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
    # Signal to indicate if manual controls should be disabled based on enum mode parameter
    diagnostics_input_mode_signal = pyqtSignal(ControlMode)

    def __init__(self, proto_unix_io: ProtoUnixIO) -> None:
        super(DiagnosticsWidget, self).__init__()

        self.proto_unix_io = proto_unix_io
        self.__control_mode = ControlMode.DIAGNOSTICS

        # initialize widgets
        self.controller_status = ControllerStatusView()
        self.controller_handler = ControllerInputHandler()
        self.drive_dribbler_widget = DriveAndDribblerWidget(proto_unix_io)
        self.chicker_widget = ChickerWidget(proto_unix_io)
        self.diagnostics_control_input_widget = DiagnosticsInputModeWidget(
            lambda control_mode: self.toggle_control(control_mode),
        )

        # self.diagnostics_control_input_widget.toggle_controls_signal.connect(
        #     lambda control_mode: self.toggle_control(control_mode)
        # )

        self.controller_refresh_button = QPushButton()
        self.controller_refresh_button.setText("Re-initialize Handheld Controller")
        self.controller_refresh_button.clicked.connect(
            self.__refresh_controller_onclick_handler
        )

        vbox_layout = QVBoxLayout()

        vbox_layout.addWidget(self.controller_refresh_button)
        vbox_layout.addWidget(self.controller_status)
        vbox_layout.addWidget(self.diagnostics_control_input_widget)
        vbox_layout.addWidget(self.drive_dribbler_widget)
        vbox_layout.addWidget(self.chicker_widget)

        self.setLayout(vbox_layout)

        self.check_controller_connection()

    def toggle_control(self, mode: ControlMode):
        self.__control_mode = mode
        self.drive_dribbler_widget.set_enabled(mode)
        self.controller_handler.toggle_controller_listener(mode)

    def refresh(self):
        if self.__control_mode == ControlMode.DIAGNOSTICS:
            self.drive_dribbler_widget.refresh()
            self.chicker_widget.refresh()

            diagnostics_primitive = Primitive(
                direct_control=DirectControlPrimitive(
                    motor_control=self.drive_dribbler_widget.motor_control,
                    power_control=self.chicker_widget.power_control,
                )
            )

            self.proto_unix_io.send_proto(Primitive, diagnostics_primitive)

        elif self.__control_mode == ControlMode.HANDHELD:
            diagnostics_primitive = (
                self.controller_handler.get_latest_primitive_command()
            )

            if diagnostics_primitive is not None:
                self.proto_unix_io.send_proto(Primitive, diagnostics_primitive)
            else:
                self.proto_unix_io.send_proto(Primitive, StopPrimitive())

    def check_controller_connection(self):
        if self.controller_handler.controller_initialized():
            # controller is connected
            logging.debug("Handheld controller is already initialized and connected...")
            self.controller_status.refresh(ControllerConnected.CONNECTED)
            self.diagnostics_control_input_widget.refresh(ControlMode.HANDHELD)

        else:
            # controller is NOT connected, try connecting...
            self.controller_handler = ControllerInputHandler()
            if self.controller_handler.controller_initialized():
                logging.debug("Successfully reinitialized handheld controller")
                self.controller_status.refresh(ControllerConnected.CONNECTED)
                self.diagnostics_control_input_widget.refresh(ControlMode.HANDHELD)

            else:
                logging.debug("Failed to reinitialize handheld controller")
                self.controller_status.refresh(ControllerConnected.DISCONNECTED)
                self.diagnostics_control_input_widget.refresh(ControlMode.DIAGNOSTICS)

    def __refresh_controller_onclick_handler(self) -> None:
        logging.debug("Attempting to reinitialize handheld controller...")
        self.check_controller_connection()

    # TODO: investigate why these methods aren't called on user hitting close button
    def closeEvent(self, event):
        logging.debug("bruh")
        self.controller_handler.close()
        event.accept()

    def __exit__(self, event):
        logging.debug("bruh")
        self.controller_handler.close()
        event.accept()
