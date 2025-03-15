from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt.QtCore import *
from proto.import_all_protos import *

from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.robot_diagnostics.chicker_widget import (
    ChickerWidget,
    ChickerCommandMode,
)
from software.thunderscope.robot_diagnostics.handheld_controller_widget import (
    HandheldControllerWidget,
)
from software.thunderscope.robot_diagnostics.drive_and_dribbler_widget import (
    DriveAndDribblerWidget,
)


class DiagnosticsWidget(QScrollArea):
    """The DiagnosticsWidget contains all widgets related to manually
    controlling robots in Robot Diagnostics:

    - HandheldControllerWidget
    - DriveAndDribblerWidget
    - ChickerWidget

    """

    def __init__(self, proto_unix_io: ProtoUnixIO) -> None:
        """Initialize the DiagnosticsWidget

        :param proto_unix_io: ProtoUnixIO (for sending messages to the robot)
                              to configure the diagnostics widgets with
        """
        super(DiagnosticsWidget, self).__init__()

        self.proto_unix_io = proto_unix_io

        self.drive_dribbler_widget = DriveAndDribblerWidget(self.proto_unix_io)
        self.chicker_widget = ChickerWidget(self.proto_unix_io)

        self.handheld_controller_widget = HandheldControllerWidget()
        self.handheld_controller_widget.kick_button_pressed.connect(
            lambda: self.chicker_widget.send_command_and_timeout(
                ChickerCommandMode.KICK
            )
        )
        self.handheld_controller_widget.chip_button_pressed.connect(
            lambda: self.chicker_widget.send_command_and_timeout(
                ChickerCommandMode.CHIP
            )
        )

        self.handheld_controller_widget.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Minimum
        )
        self.drive_dribbler_widget.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding
        )
        self.chicker_widget.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding
        )

        diagnostics_widget_vbox_layout = QVBoxLayout()
        diagnostics_widget_vbox_layout.addWidget(self.handheld_controller_widget)
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

    def refresh(self) -> None:
        """Call the refresh function on all sub-widgets.

        If controller input is enabled in the HandheldControllerWidget, visually update the
        DriveAndDribblerWidget and the ChickerWidget to display the motor and chicker values
        currently set by the handheld controller.
        """
        self.handheld_controller_widget.refresh()

        if self.handheld_controller_widget.controller_input_enabled():
            self.chicker_widget.disable()
            self.drive_dribbler_widget.disable()

            self.drive_dribbler_widget.override_slider_values(
                self.handheld_controller_widget.motor_control
            )
            self.chicker_widget.override_slider_values(
                self.handheld_controller_widget.kick_power,
                self.handheld_controller_widget.chip_distance,
            )
        else:
            self.chicker_widget.enable()
            self.drive_dribbler_widget.enable()

        self.drive_dribbler_widget.refresh()
        self.chicker_widget.refresh()
