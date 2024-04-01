from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt.QtCore import *
from proto.import_all_protos import *
from software.logger.logger import createLogger

from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.robot_diagnostics.chicker_widget import ChickerWidget
from software.thunderscope.robot_diagnostics.handheld_device_status_view import (
    HandheldDeviceStatusView,
    ControllerConnectionState,
)
from software.thunderscope.robot_diagnostics.diagnostics_input_widget import (
    DiagnosticsInputToggleWidget,
    ControlMode,
)
from software.thunderscope.robot_diagnostics.handheld_device_manager import (
    HandheldDeviceManager,
)
from software.thunderscope.robot_diagnostics.drive_and_dribbler_widget import (
    DriveAndDribblerWidget,
)


class DiagnosticsWidget(QWidget):
    # Signal to indicate if manual controls should be disabled based on enum mode parameter
    diagnostics_input_mode_signal = pyqtSignal(ControlMode)

    def __init__(self, proto_unix_io: ProtoUnixIO) -> None:
        super(DiagnosticsWidget, self).__init__()

        self.logger = createLogger("RobotDiagnostics")

        self.proto_unix_io = proto_unix_io
        self.__control_mode = ControlMode.DIAGNOSTICS

        # initialize widgets
        self.controller_status = HandheldDeviceStatusView()
        self.drive_dribbler_widget = DriveAndDribblerWidget(proto_unix_io)
        self.chicker_widget = ChickerWidget(proto_unix_io)
        self.diagnostics_control_input_widget = DiagnosticsInputToggleWidget(
            lambda control_mode: self.toggle_control(control_mode),
        )

        # initialize controller
        self.controller_handler = HandheldDeviceManager(self.logger)

        # initialize controller refresh button
        self.controller_refresh_button = QPushButton()
        self.controller_refresh_button.setText("Re-initialize Handheld Controller")
        self.controller_refresh_button.clicked.connect(
            self.__refresh_controller_onclick_handler
        )

        # TODO: move to another class.
        # Layout for the entire diagnostics tab
        vbox_layout = QVBoxLayout()

        # Layout for controller button & status
        hbox_layout = QHBoxLayout()

        hbox_layout.addWidget(self.controller_status)
        hbox_layout.addWidget(self.controller_refresh_button)
        hbox_layout.setStretch(0, 4)
        hbox_layout.setStretch(1, 1)

        vbox_layout.addLayout(hbox_layout)

        vbox_layout.addWidget(self.diagnostics_control_input_widget)
        vbox_layout.addWidget(self.drive_dribbler_widget)
        vbox_layout.addWidget(self.chicker_widget)

        self.setLayout(vbox_layout)

    def toggle_control(self, mode: ControlMode):
        self.__control_mode = mode

        self.drive_dribbler_widget.refresh(mode)
        self.chicker_widget.refresh(mode)
        self.controller_handler.refresh(mode)

    def refresh(self):
        """
        Refreshes sub-widgets so that they display the most recent values, as well as
        the most recent values are available for use.
        """
        controller_status = self.controller_handler.get_controller_connection_status()
        self.controller_status.refresh(controller_status)
        self.diagnostics_control_input_widget.refresh(controller_status)

        mode = self.__control_mode
        self.drive_dribbler_widget.refresh(mode)
        self.chicker_widget.refresh(mode)

        if self.__control_mode == ControlMode.DIAGNOSTICS:
            diagnostics_primitive = Primitive(
                direct_control=DirectControlPrimitive(
                    motor_control=self.drive_dribbler_widget.motor_control,
                    power_control=self.chicker_widget.power_control,
                )
            )

            self.proto_unix_io.send_proto(Primitive, diagnostics_primitive)

        elif self.__control_mode == ControlMode.HANDHELD:
            diagnostics_primitive = (
                self.controller_handler.get_latest_primitive_controls()
            )

            if diagnostics_primitive is not None:
                self.proto_unix_io.send_proto(Primitive, diagnostics_primitive)
            else:
                self.proto_unix_io.send_proto(Primitive, StopPrimitive())

    def __refresh_controller_onclick_handler(self) -> None:
        self.controller_handler.initialize_controller()

    # TODO: investigate why these methods aren't called on user hitting close button
    def closeEvent(self, event):
        self.controller_handler.close()
        event.accept()

    def __exit__(self, event):
        self.controller_handler.close()
        event.accept()
