from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt.QtCore import *
from proto.import_all_protos import *
from software.logger.logger import createLogger

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
    DriveAndDribblerWidget, SliderType,
)


class DiagnosticsWidget(QWidget):
    # signal to indicate if manual controls should be disabled based on enum mode parameter
    diagnostics_input_mode_signal = pyqtSignal(ControlMode)

    # signal to indicate that controller should be reinitialized
    reinitialize_controller_signal = pyqtSignal()

    def __init__(self, proto_unix_io: ProtoUnixIO) -> None:
        super(DiagnosticsWidget, self).__init__()

        self.logger = createLogger("RobotDiagnostics")

        self.proto_unix_io = proto_unix_io

        # default start with diagnostics input
        self.__control_mode = ControlMode.DIAGNOSTICS

        # initialize widgets
        self.controller_status = HandheldDeviceStatusView(self.reinitialize_controller_signal)
        self.drive_dribbler_widget = DriveAndDribblerWidget()
        self.chicker_widget = ChickerWidget(proto_unix_io)
        self.diagnostics_control_input_widget = DiagnosticsInputToggleWidget(
            self.diagnostics_input_mode_signal
        )

        # connect input mode signal with handler
        self.diagnostics_input_mode_signal.connect(self.__toggle_control_and_refresh)

        # connect controller reinitialization signal with handler
        self.reinitialize_controller_signal.connect(self.__reinitialize_controller)

        # initialize controller
        self.controller_handler = HandheldDeviceManager(proto_unix_io, self.logger)

        # layout for the entire diagnostics tab
        vbox_layout = QVBoxLayout()

        vbox_layout.addWidget(self.controller_status)
        vbox_layout.addWidget(self.diagnostics_control_input_widget)
        vbox_layout.addWidget(self.drive_dribbler_widget)
        vbox_layout.addWidget(self.chicker_widget)

        self.setLayout(vbox_layout)

    def __toggle_control_and_refresh(self, mode: ControlMode):
        self.__control_mode = mode

        self.drive_dribbler_widget.refresh(mode)
        self.chicker_widget.refresh(mode)
        self.controller_handler.refresh(mode)

    def refresh(self):
        """
        Refreshes sub-widgets so that they display the most recent values, as well as
        the most recent values are available for use.
        """
        # get the most recent state on whether controller is connected from controller handler
        controller_status = self.controller_handler.get_controller_connection_status()

        # update controller status view with most recent controller status
        self.controller_status.refresh(controller_status)

        # update diagnostic ui with most recent recent controller status
        self.diagnostics_control_input_widget.refresh(controller_status)

        if controller_status == HandheldDeviceConnectionStatus.DISCONNECTED:
            self.__toggle_control_and_refresh(ControlMode.DIAGNOSTICS)

        if self.__control_mode == ControlMode.DIAGNOSTICS:
            diagnostics_primitive = Primitive(
                direct_control=DirectControlPrimitive(
                    motor_control=self.drive_dribbler_widget.motor_control,
                    power_control=self.chicker_widget.power_control,
                )
            )

            self.proto_unix_io.send_proto(Primitive, diagnostics_primitive)

        elif self.__control_mode == ControlMode.HANDHELD:
            # diagnostics_primitive = (
            #     self.controller_handler.get_latest_primitive_controls()
            # )

            self.drive_dribbler_widget.set_slider(
                SliderType.XVelocitySlider,
                self.controller_handler.move_x
            )
            self.drive_dribbler_widget.set_slider(
                SliderType.YVelocitySlider,
                self.controller_handler.move_y
            )
            self.drive_dribbler_widget.set_slider(
                SliderType.DribblerVelocitySlider,
                self.controller_handler.ang_vel
            )

            # if diagnostics_primitive is not None:
            #     self.proto_unix_io.send_proto(Primitive, diagnostics_primitive)
            # else:
            #     self.proto_unix_io.send_proto(Primitive, StopPrimitive())

    def __reinitialize_controller(self) -> None:
        self.controller_handler.clear_controller()
        self.controller_handler.initialize_controller()

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
