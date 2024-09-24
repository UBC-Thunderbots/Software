from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt.QtCore import *
from proto.import_all_protos import *
from software.logger.logger import create_logger

from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.robot_diagnostics.chicker_widget import ChickerWidget
from software.thunderscope.robot_diagnostics.handheld_device_status_view import (
    HandheldDeviceStatusView,
)
from software.thunderscope.robot_diagnostics.diagnostics_input_widget import (
    DiagnosticsInputToggleWidget,
    ControlMode,
)
from software.thunderscope.robot_diagnostics.handheld_device_manager import (
    HandheldDeviceManager,
    HandheldDeviceConnectionChangedEvent,
)
from software.thunderscope.robot_diagnostics.drive_and_dribbler_widget import (
    DriveAndDribblerWidget,
)


class DiagnosticsWidget(QScrollArea):
    """The DiagnosticsWidget contains all widgets related to Robot Diagnostics:

    - HandheldDeviceStatusWidget
    - DiagnosticsInputWidget
    - DriveAndDribblerWidget
    - ChickerWidget

    """

    def __init__(self, proto_unix_io: ProtoUnixIO) -> None:
        """Initialize the DiagnosticsWidget

        :param proto_unix_io: ProtoUnixIO (for sending messages to the robot)
                              to configure the diagnostics widgets with
        """
        super(DiagnosticsWidget, self).__init__()

        self.logger = create_logger("RobotDiagnostics")

        self.proto_unix_io = proto_unix_io

        self.handheld_device_status_widget = HandheldDeviceStatusView()
        self.drive_dribbler_widget = DriveAndDribblerWidget(self.proto_unix_io)
        self.chicker_widget = ChickerWidget(self.proto_unix_io)
        self.diagnostics_control_input_widget = DiagnosticsInputToggleWidget()

        self.handheld_device_manager = HandheldDeviceManager(
            self.logger,
            self.proto_unix_io,
        )

        self.handheld_device_manager.handheld_device_connection_status_signal.connect(
            self.__handheld_device_connection_status_signal_handler
        )

        self.diagnostics_control_input_widget.control_mode_changed_signal.connect(
            self.__control_mode_changed_signal_handler
        )

        self.handheld_device_status_widget.reinitialize_handheld_device_signal.connect(
            self.handheld_device_manager.reinitialize_handheld_device
        )

        self.handheld_device_manager.reinitialize_handheld_device()

        diagnostics_widget_vbox_layout = QVBoxLayout()
        diagnostics_widget_vbox_layout.addWidget(self.handheld_device_status_widget)
        diagnostics_widget_vbox_layout.addWidget(self.diagnostics_control_input_widget)
        diagnostics_widget_vbox_layout.addWidget(self.drive_dribbler_widget)
        diagnostics_widget_vbox_layout.addWidget(self.chicker_widget)

        # for a QScrollArea, widgets cannot be added to it directly
        # doing so causes no scrolling to happen, and all the components get smaller
        # instead, widgets are added to the layout which is set for a container
        # the container is set as the current QScrollArea's widget
        self.container = QFrame(self)
        self.container.setLayout(diagnostics_widget_vbox_layout)
        self.setWidget(self.container)
        self.setWidgetResizable(True)

    def __control_mode_changed_signal_handler(self, mode: ControlMode) -> None:
        """Handler for the control_mode_changed_signal emitted by DiagnosticsControlInputWidget

        :param mode: the currently selected control mode
        """
        self.handheld_device_manager.set_enabled(mode == ControlMode.HANDHELD)
        self.drive_dribbler_widget.update_widget_accessibility(mode)
        self.chicker_widget.update_widget_accessibility(mode)

    def __handheld_device_connection_status_signal_handler(
        self, event: HandheldDeviceConnectionChangedEvent
    ) -> None:
        """Handler for the handheld_device_connection_status_signal emitted by HandheldDeviceManager

        :param event: the signal event containing the new handheld device connection status
        """
        self.handheld_device_status_widget.update(
            event.connection_status, event.device_name
        )
        self.diagnostics_control_input_widget.update(event.connection_status)

    def refresh(self) -> None:
        """Refresh sub-widgets so that they display the most recent status values.
        If in handheld mode, visually update driver, dribbler and chicker sliders
        to the values currently being set by the handheld device.
        """
        control_mode = self.diagnostics_control_input_widget.get_control_mode()

        self.chicker_widget.refresh()

        if control_mode == ControlMode.DIAGNOSTICS:
            self.drive_dribbler_widget.refresh()

        elif control_mode == ControlMode.HANDHELD:
            with self.handheld_device_manager.lock:
                motor_control = self.handheld_device_manager.motor_control
                kick_power = self.handheld_device_manager.kick_power
                chip_distance = self.handheld_device_manager.chip_distance

                self.drive_dribbler_widget.set_x_velocity_slider(
                    motor_control.direct_velocity_control.velocity.x_component_meters
                )
                self.drive_dribbler_widget.set_y_velocity_slider(
                    motor_control.direct_velocity_control.velocity.y_component_meters
                )
                self.drive_dribbler_widget.set_angular_velocity_slider(
                    motor_control.direct_velocity_control.angular_velocity.radians_per_second
                )

                self.drive_dribbler_widget.set_dribbler_velocity_slider(
                    motor_control.dribbler_speed_rpm
                )

                self.chicker_widget.set_kick_power_slider_value(kick_power)
                self.chicker_widget.set_chip_distance_slider_value(chip_distance)
