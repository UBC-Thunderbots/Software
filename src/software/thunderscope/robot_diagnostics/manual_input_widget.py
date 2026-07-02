import numpy

from proto.import_all_protos import *
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt import QtCore

import software.python_bindings as tbots_cpp

from software.thunderscope.constants import DiagnosticsConstants


class ManualInputWidget(QWidget):
    """Base class for widgets that let the user manually drive a robot from an
    input device (e.g. a handheld controller or the keyboard).

    Subclasses gather a small, device-independent set of inputs from their
    device and call :meth:`_apply_inputs`, which holds the shared logic for
    translating those inputs into a MotorControl message along with the
    kick/chip/dribbler settings. This keeps the input-to-command mapping
    identical across every input source.
    """

    kick_button_pressed = QtCore.pyqtSignal()
    """Signal emitted when the kick button/key is pressed"""

    chip_button_pressed = QtCore.pyqtSignal()
    """Signal emitted when the chip button/key is pressed"""

    def __init__(self) -> None:
        """Initialize the ManualInputWidget."""
        super().__init__()

        self.constants = tbots_cpp.createRobotConstants()

        self.motor_control = MotorControl()
        self.dribbler_speed = 0
        self.kick_power = DiagnosticsConstants.MIN_KICK_POWER
        self.chip_distance = DiagnosticsConstants.MIN_CHIP_POWER

        self.last_d_pad_axis_x_value = 0
        self.last_d_pad_axis_y_value = 0
        self.last_kick_value = False
        self.last_chip_value = False

    def poll(self) -> None:
        """Read the current input state from the device and update the motor
        control / kick / chip / dribbler values accordingly.

        Implemented by subclasses; only called while this widget is the active
        input source.
        """
        raise NotImplementedError

    @staticmethod
    def _with_deadzone(value: float) -> float:
        """Zero out an axis value that falls within the deadzone.

        :param value: the normalized axis value in [-1, 1]
        :return: the value, or 0 if it is within the deadzone
        """
        return (
            value if abs(value) >= DiagnosticsConstants.DEADZONE_PERCENTAGE else 0
        )

    def _apply_inputs(
        self,
        move_x: float,
        move_y: float,
        move_rot: float,
        slow_mode: bool,
        dribbler_on: bool,
        d_pad_axis_x: int,
        d_pad_axis_y: int,
        kick: bool,
        chip: bool,
    ) -> None:
        """Translate a device-independent set of inputs into a MotorControl
        message and the current kick/chip/dribbler settings, emitting the
        kick/chip signals on a rising edge.

        The movement axes follow the same sign convention as the handheld
        controller's analog sticks; see each subclass for how its device maps
        onto them.

        :param move_x: forward/backward translation axis, normalized to [-1, 1]
        :param move_y: left/right translation axis, normalized to [-1, 1]
        :param move_rot: rotation axis, normalized to [-1, 1]
        :param slow_mode: whether to scale velocities down for fine control
        :param dribbler_on: whether the dribbler should spin at its set speed
        :param d_pad_axis_x: -1, 0 or 1; steps the kick power and chip distance
        :param d_pad_axis_y: -1, 0 or 1; steps the dribbler speed
        :param kick: whether the kick button/key is currently pressed
        :param chip: whether the chip button/key is currently pressed
        """
        move_x = self._with_deadzone(move_x)
        move_y = self._with_deadzone(move_y)
        move_rot = self._with_deadzone(move_rot)

        speed_factor = (
            DiagnosticsConstants.SPEED_SLOWDOWN_FACTOR if slow_mode else 1
        )

        self.motor_control.direct_velocity_control.velocity.x_component_meters = (
            -move_x * self.constants.robot_max_speed_m_per_s * speed_factor
        )
        self.motor_control.direct_velocity_control.velocity.y_component_meters = (
            -move_y * self.constants.robot_max_speed_m_per_s * speed_factor
        )
        self.motor_control.direct_velocity_control.angular_velocity.radians_per_second = (
            -move_rot * self.constants.robot_max_ang_speed_rad_per_s * speed_factor
        )

        if d_pad_axis_x != self.last_d_pad_axis_x_value:
            self.last_d_pad_axis_x_value = d_pad_axis_x
            self.kick_power = numpy.clip(
                a=self.kick_power
                + int(d_pad_axis_x) * DiagnosticsConstants.KICK_POWER_STEPPER,
                a_min=DiagnosticsConstants.MIN_KICK_POWER,
                a_max=DiagnosticsConstants.MAX_KICK_POWER,
            )
            self.chip_distance = numpy.clip(
                a=self.chip_distance
                + d_pad_axis_x * DiagnosticsConstants.CHIP_DISTANCE_STEPPER,
                a_min=DiagnosticsConstants.MIN_CHIP_POWER,
                a_max=DiagnosticsConstants.MAX_CHIP_POWER,
            )

        if d_pad_axis_y != self.last_d_pad_axis_y_value:
            self.last_d_pad_axis_y_value = d_pad_axis_y
            self.dribbler_speed = int(
                numpy.clip(
                    a=self.dribbler_speed
                    - d_pad_axis_y * DiagnosticsConstants.DRIBBLER_RPM_STEPPER,
                    a_min=self.constants.indefinite_dribbler_speed_rpm,
                    a_max=-self.constants.indefinite_dribbler_speed_rpm,
                )
            )

        self.motor_control.dribbler_speed_rpm = (
            self.dribbler_speed if dribbler_on else 0
        )

        if kick != self.last_kick_value:
            self.last_kick_value = kick
            if kick:
                self.kick_button_pressed.emit()

        if chip != self.last_chip_value:
            self.last_chip_value = chip
            if chip:
                self.chip_button_pressed.emit()
