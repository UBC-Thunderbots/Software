from enum import IntEnum

from proto.import_all_protos import *
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt import QtCore

from software.thunderscope.constants import DiagnosticsConstants
from software.thunderscope.robot_diagnostics.handheld_controller_widget import (
    HandheldControllerWidget,
)
from software.thunderscope.robot_diagnostics.keyboard_controller_widget import (
    KeyboardControllerWidget,
)
from software.thunderscope.robot_diagnostics.manual_input_widget import (
    ManualInputWidget,
)


class ManualInputSource(IntEnum):
    """The input sources the user can select to manually control a robot.

    The values double as the page indices of the source selector's
    QStackedWidget.
    """

    NONE = 0
    CONTROLLER = 1
    KEYBOARD = 2


class ManualControlWidget(QWidget):
    """Lets the user choose how to manually control robots in diagnostics:
    using a handheld game controller, the keyboard, or neither.

    Exposes a uniform interface (motor_control, kick_power, chip_distance, the
    kick/chip signals and input_enabled()) over whichever source is currently
    selected, so the rest of diagnostics doesn't need to know which device is
    in use.
    """

    kick_button_pressed = QtCore.pyqtSignal()
    """Signal emitted when the kick button/key is pressed on the active source"""

    chip_button_pressed = QtCore.pyqtSignal()
    """Signal emitted when the chip button/key is pressed on the active source"""

    def __init__(self) -> None:
        """Initialize the ManualControlWidget."""
        super().__init__()

        self.controller_widget = HandheldControllerWidget()
        self.keyboard_widget = KeyboardControllerWidget()

        # Re-emit the kick/chip signals from whichever source fires them; only
        # the active source is ever polled, so the inactive one stays silent.
        for source in (self.controller_widget, self.keyboard_widget):
            source.kick_button_pressed.connect(self.kick_button_pressed)
            source.chip_button_pressed.connect(self.chip_button_pressed)

        self.input_source_combo_box = QComboBox()
        self.input_source_combo_box.addItem("None", ManualInputSource.NONE)
        self.input_source_combo_box.addItem("Controller", ManualInputSource.CONTROLLER)
        self.input_source_combo_box.addItem("Keyboard", ManualInputSource.KEYBOARD)

        # Pages must be added in ManualInputSource order so the enum value is
        # also the stack page index. NONE shows an empty placeholder.
        self.source_stack = QStackedWidget()
        self.source_stack.addWidget(QWidget())
        self.source_stack.addWidget(self.controller_widget)
        self.source_stack.addWidget(self.keyboard_widget)

        self.input_source_combo_box.currentIndexChanged.connect(
            self.__on_source_changed
        )

        selector_layout = QHBoxLayout()
        selector_layout.addWidget(QLabel("Input Source:"))
        selector_layout.addWidget(self.input_source_combo_box, stretch=1)

        box_layout = QVBoxLayout()
        box_layout.addLayout(selector_layout)
        box_layout.addWidget(self.source_stack)

        box = QGroupBox()
        box.setTitle("Manual Control")
        box.setLayout(box_layout)

        widget_layout = QVBoxLayout()
        widget_layout.addWidget(box)
        self.setLayout(widget_layout)

    def refresh(self) -> None:
        """Refresh the selected input source and read its current inputs.

        Keeps the controller's connection status up to date while it is the
        selected source, and polls the active source so motor_control and the
        kick/chip/dribbler settings reflect the latest inputs.
        """
        if self.__current_source() == ManualInputSource.CONTROLLER:
            self.controller_widget.refresh_status()

        if self.input_enabled():
            self.__active_widget().poll()

    def input_enabled(self) -> bool:
        """Check whether a manual input source is actively driving the robot.

        Keyboard input is always active once selected; controller input is only
        active while a controller is actually connected.

        :return: true if a manual input source is driving the robot
        """
        source = self.__current_source()
        if source == ManualInputSource.CONTROLLER:
            return self.controller_widget.is_connected()
        return source == ManualInputSource.KEYBOARD

    @property
    def motor_control(self) -> MotorControl:
        """The MotorControl set by the active input source."""
        active = self.__active_widget()
        return active.motor_control if active else MotorControl()

    @property
    def kick_power(self) -> float:
        """The kick power set by the active input source."""
        active = self.__active_widget()
        return active.kick_power if active else DiagnosticsConstants.MIN_KICK_POWER

    @property
    def chip_distance(self) -> float:
        """The chip distance set by the active input source."""
        active = self.__active_widget()
        return active.chip_distance if active else DiagnosticsConstants.MIN_CHIP_POWER

    def __current_source(self) -> ManualInputSource:
        """:return: the currently selected input source"""
        return self.input_source_combo_box.currentData()

    def __active_widget(self) -> ManualInputWidget | None:
        """:return: the widget for the selected source, or None if NONE is selected"""
        source = self.__current_source()
        if source == ManualInputSource.CONTROLLER:
            return self.controller_widget
        if source == ManualInputSource.KEYBOARD:
            return self.keyboard_widget
        return None

    def __on_source_changed(self) -> None:
        """Switch the visible source panel and (de)activate keyboard capture."""
        source = self.__current_source()
        self.source_stack.setCurrentIndex(int(source))
        self.keyboard_widget.set_active(source == ManualInputSource.KEYBOARD)
