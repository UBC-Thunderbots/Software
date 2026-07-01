from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt.QtCore import *
from proto.import_all_protos import *

from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.robot_diagnostics.chicker_widget import (
    ChickerWidget,
    ChickerCommandMode,
)
from software.thunderscope.robot_diagnostics.manual_control_widget import (
    ManualControlWidget,
)
from software.thunderscope.robot_diagnostics.drive_and_dribbler_widget import (
    DriveAndDribblerWidget,
)


class DiagnosticsWidget(QScrollArea):
    """The DiagnosticsWidget contains all widgets related to manually
    controlling robots in Robot Diagnostics:

    - ManualControlWidget
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

        self.manual_control_widget = ManualControlWidget()
        self.manual_control_widget.kick_button_pressed.connect(
            lambda: self.chicker_widget.send_command_and_timeout(
                ChickerCommandMode.KICK
            )
        )
        self.manual_control_widget.chip_button_pressed.connect(
            lambda: self.chicker_widget.send_command_and_timeout(
                ChickerCommandMode.CHIP
            )
        )

        self.manual_control_widget.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Minimum
        )
        self.drive_dribbler_widget.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding
        )
        self.chicker_widget.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding
        )

        diagnostics_widget_vbox_layout = QVBoxLayout()
        diagnostics_widget_vbox_layout.addWidget(self.manual_control_widget)
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

        If a manual input source is enabled in the ManualControlWidget, visually update the
        DriveAndDribblerWidget and the ChickerWidget to display the motor and chicker values
        currently set by that input source.
        """
        self.manual_control_widget.refresh()

        if self.manual_control_widget.input_enabled():
            self.chicker_widget.disable()
            self.drive_dribbler_widget.disable()

            self.drive_dribbler_widget.override_slider_values(
                self.manual_control_widget.motor_control
            )
            self.chicker_widget.override_slider_values(
                self.manual_control_widget.kick_power,
                self.manual_control_widget.chip_distance,
            )
        else:
            self.chicker_widget.enable()
            self.drive_dribbler_widget.enable()

        self.drive_dribbler_widget.refresh()
        self.chicker_widget.refresh()
