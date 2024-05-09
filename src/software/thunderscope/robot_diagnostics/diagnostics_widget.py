from typing import Union, Literal, Tuple

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
    """
    The DiagnosticsWidget contains all widgets related to diagnostics:
        - HandheldDeviceStatusWidget
        - DiagnosticsInputWidget
        - DriveAndDribblerWidget
        - ChickerWidget

    This widget is also responsible for controlling the diagnostics inout,
    either from the DriverAndDribblerWidget or the HandheldDeviceManager,
    depending on the user selected choice from the DiagnosticsWidget UI
    """
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
        self.drive_dribbler_widget = DriveAndDribblerWidget(self.proto_unix_io)
        self.chicker_widget = ChickerWidget(self.proto_unix_io)
        self.diagnostics_control_input_widget = DiagnosticsInputToggleWidget(
            self.diagnostics_input_mode_signal
        )

        # connect input mode signal with handler
        self.diagnostics_input_mode_signal.connect(self.__control_mode_update_handler)

        # connect handheld device reinitialization signal with handler
        self.reinitialize_handheld_device_signal.connect(
            lambda: self.handheld_device_handler.reinitialize_controller()
        )

        # connect handheld device connection status toggle signal with handler
        self.handheld_device_connection_status_signal.connect(
            self.__handheld_device_connection_status_update_handler
        )

        # initialize controller
        self.handheld_device_handler = HandheldDeviceManager(
            self.logger,
            self.proto_unix_io,
            self.handheld_device_connection_status_signal,
        )

        # layout for the entire diagnostics tab
        vbox_layout = QVBoxLayout()

        vbox_layout.addWidget(self.handheld_device_status_widget)
        vbox_layout.addWidget(self.diagnostics_control_input_widget)
        vbox_layout.addWidget(self.drive_dribbler_widget)
        vbox_layout.addWidget(self.chicker_widget)

        self.setLayout(vbox_layout)

    def __control_mode_update_handler(self, mode: ControlMode) -> None:
        """
        Handler for managing a control mode update
        :param mode: The new mode that is being used
        """
        self.__control_mode = mode
        self.handheld_device_handler.refresh(self.__control_mode)

    def __handheld_device_connection_status_update_handler(
        self, status: HandheldDeviceConnectionStatus
    ):
        """
        Handler for propagating
        :param status: The new mode that is being used
        """
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
            self.logger.debug(self.chicker_widget.power_control)

        elif self.__control_mode == ControlMode.HANDHELD:
            self.logger.debug(self.handheld_device_handler.power_control)

            # update drive and dribbler visually.
            # TODO kick power, flash kick and chip buttons
            self.drive_dribbler_widget.set_x_velocity_slider(
                self.handheld_device_handler.motor_control.direct_velocity_control.velocity.x_component_meters
            )
            self.drive_dribbler_widget.set_y_velocity_slider(
                self.handheld_device_handler.motor_control.direct_velocity_control.velocity.y_component_meters
            )
            self.drive_dribbler_widget.set_angular_velocity_slider(
                self.handheld_device_handler.motor_control.direct_velocity_control.angular_velocity.radians_per_second
            )

            self.drive_dribbler_widget.set_dribbler_velocity_slider(
                self.handheld_device_handler.motor_control.dribbler_speed_rpm
            )

    # TODO: investigate why these methods aren't called on user hitting close button
    def closeEvent(self, event):
        self.logger.info("test")
        self.handheld_device_handler.close()
        event.accept()

    def close(self, event):
        self.logger.info("test")
        self.handheld_device_handler.close()
        event.accept()

    def __exit__(self, event):
        self.logger.info("test")
        self.handheld_device_handler.close()
        event.accept()
