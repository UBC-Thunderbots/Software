from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt.QtCore import *
from proto.import_all_protos import *
from software.logger.logger import create_logger

from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.robot_diagnostics.chicker_widget import ChickerWidget
from software.thunderscope.robot_diagnostics.handheld_device_status_view import (
    HandheldDeviceStatusView,
    HandheldDeviceConnectionStatus,
)
from software.thunderscope.robot_diagnostics.diagnostics_input_widget import (
    DiagnosticsInputToggleWidget,
    ControlMode,
)
from software.thunderscope.robot_diagnostics.handheld_device_manager import (
    HandheldDeviceManager,
)
from software.thunderscope.robot_diagnostics.drive_and_dribbler_widget import (
    DriveAndDribblerWidget
)


class DiagnosticsWidget(QWidget):
    # signal to indicate if manual controls should be disabled based on enum mode parameter
    diagnostics_input_mode_signal = pyqtSignal(ControlMode)

    # signal to indicate that controller should be reinitialized
    reinitialize_handheld_device_signal = pyqtSignal()

    # signal to indicate that controller was disconnected
    handheld_device_connection_status_signal = pyqtSignal(
        HandheldDeviceConnectionStatus
    )

    def __init__(self, proto_unix_io: ProtoUnixIO) -> None:
        super(DiagnosticsWidget, self).__init__()

        self.logger = create_logger("RobotDiagnostics")

        self.proto_unix_io = proto_unix_io

        # default start with diagnostics input
        self.__control_mode = ControlMode.DIAGNOSTICS

        # default start with disconnected controller
        self.__handheld_device_status = HandheldDeviceConnectionStatus.DISCONNECTED

        # initialize widgets
        self.handheld_device_status_widget = HandheldDeviceStatusView(
            self.reinitialize_handheld_device_signal
        )
        self.drive_dribbler_widget = DriveAndDribblerWidget()
        self.chicker_widget = ChickerWidget(proto_unix_io)
        self.diagnostics_control_input_widget = DiagnosticsInputToggleWidget(
            self.diagnostics_input_mode_signal
        )

        # connect input mode signal with handler
        self.diagnostics_input_mode_signal.connect(self.__toggle_control)

        # connect handheld device reinitialization signal with handler
        self.reinitialize_handheld_device_signal.connect(self.__reinitialize_controller)

        # connect handheld device connection status toggle signal with handler
        self.handheld_device_connection_status_signal.connect(
            self.__toggle_handheld_device_connection_status
        )

        # initialize controller
        self.controller_handler = HandheldDeviceManager(
            self.proto_unix_io,  # TODO: proto shouldn't be passed down
            self.logger,
            self.handheld_device_connection_status_signal,
        )

        # layout for the entire diagnostics tab
        vbox_layout = QVBoxLayout()

        vbox_layout.addWidget(self.handheld_device_status_widget)
        vbox_layout.addWidget(self.diagnostics_control_input_widget)
        vbox_layout.addWidget(self.drive_dribbler_widget)
        vbox_layout.addWidget(self.chicker_widget)

        self.setLayout(vbox_layout)

    def __toggle_control(self, mode: ControlMode):
        self.__control_mode = mode
        self.controller_handler.refresh(self.__control_mode)

    def __toggle_handheld_device_connection_status(
        self, status: HandheldDeviceConnectionStatus
    ):
        self.__handheld_device_status = status

    def refresh(self):
        """
        Refreshes sub-widgets so that they display the most recent values, as well as
        the most recent values are available for use.
        """

        # update controller status view with most recent controller status
        self.handheld_device_status_widget.refresh(self.__handheld_device_status)

        # update diagnostic ui with most recent recent controller status
        self.diagnostics_control_input_widget.refresh(self.__handheld_device_status)

        # update all widgets based on current control mode
        self.drive_dribbler_widget.refresh(self.__control_mode)
        self.chicker_widget.refresh(self.__control_mode)

        if self.__control_mode == ControlMode.DIAGNOSTICS:
            diagnostics_primitive = Primitive(
                direct_control=DirectControlPrimitive(
                    motor_control=self.drive_dribbler_widget.motor_control,
                    power_control=self.chicker_widget.power_control,
                )
            )
            # self.logger.debug(self.drive_dribbler_widget.motor_control)
            self.proto_unix_io.send_proto(Primitive, diagnostics_primitive)

        elif self.__control_mode == ControlMode.HANDHELD:
            # get latest diagnostics primitive
            diagnostics_primitive = (
                self.controller_handler.get_latest_primitive_controls()
            )

            # send it

            # update ui visually
            # TODO update dribbler speed and kick power, flash kick and chip buttons
            self.drive_dribbler_widget.set_x_velocity_slider(
                diagnostics_primitive.direct_control.motor_control.direct_velocity_control.velocity.x_component_meters
            )
            self.drive_dribbler_widget.set_y_velocity_slider(
                diagnostics_primitive.direct_control.motor_control.direct_velocity_control.velocity.y_component_meters
            )
            self.drive_dribbler_widget.set_angular_velocity_slider(
                diagnostics_primitive.direct_control.motor_control.direct_velocity_control.angular_velocity.radians_per_second
            )

            self.drive_dribbler_widget.set_dribbler_velocity_slider(
                diagnostics_primitive.direct_control.motor_control.dribbler_speed_rpm
            )

            self.proto_unix_io.send_proto(Primitive, diagnostics_primitive)

    def __reinitialize_controller(self) -> None:
        self.controller_handler.reinitialize_controller()

    # TODO: investigate why these methods aren't called on user hitting close button
    def closeEvent(self, event):
        self.logger.info("test")
        self.controller_handler.close()
        event.accept()

    def close(self, event):
        self.logger.info("test")
        self.controller_handler.close()
        event.accept()

    def __exit__(self, event):
        self.logger.info("test")
        self.controller_handler.close()
        event.accept()
