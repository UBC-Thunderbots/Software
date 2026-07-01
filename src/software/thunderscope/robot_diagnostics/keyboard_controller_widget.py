from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt import QtGui
from pyqtgraph.Qt.QtCore import Qt

from software.thunderscope.robot_diagnostics.manual_input_widget import (
    ManualInputWidget,
)


class KeyboardControllerWidget(ManualInputWidget):
    """Input source that drives a robot from the keyboard.

    While this source is active, the widget grabs the keyboard so the mapped
    keys control the robot regardless of which widget currently has focus
    (selecting a different input source releases the keyboard again). Movement
    keys follow the same sign convention as the handheld controller's analog
    sticks, so the input-to-command mapping in
    :meth:`ManualInputWidget._apply_inputs` is shared between the two.
    """

    # Translation (WASD) and rotation (Q/E)
    _KEY_FORWARD = Qt.Key.Key_W
    _KEY_BACKWARD = Qt.Key.Key_S
    _KEY_LEFT = Qt.Key.Key_A
    _KEY_RIGHT = Qt.Key.Key_D
    _KEY_ROTATE_LEFT = Qt.Key.Key_Q
    _KEY_ROTATE_RIGHT = Qt.Key.Key_E

    # Modifiers / actions
    _KEY_SLOW = Qt.Key.Key_Shift
    _KEY_KICK = Qt.Key.Key_Space
    _KEY_CHIP = Qt.Key.Key_C
    _KEY_DRIBBLE = Qt.Key.Key_F

    # Arrow keys step the kick power / chip distance (left/right) and the
    # dribbler speed (up/down), mirroring the controller's D-pad
    _KEY_KICK_POWER_UP = Qt.Key.Key_Right
    _KEY_KICK_POWER_DOWN = Qt.Key.Key_Left
    _KEY_DRIBBLER_UP = Qt.Key.Key_Up
    _KEY_DRIBBLER_DOWN = Qt.Key.Key_Down

    _LEGEND = (
        "Move: W / S forward / back, A / D strafe left / right\n"
        "Rotate: Q / E      Slow mode: hold Shift\n"
        "Kick: Space      Chip: C      Dribbler: hold F\n"
        "Kick power: ← / →      Dribbler speed: ↑ / ↓"
    )

    def __init__(self) -> None:
        """Initialize the KeyboardControllerWidget."""
        super().__init__()

        self._tracked_keys = {
            self._KEY_FORWARD,
            self._KEY_BACKWARD,
            self._KEY_LEFT,
            self._KEY_RIGHT,
            self._KEY_ROTATE_LEFT,
            self._KEY_ROTATE_RIGHT,
            self._KEY_SLOW,
            self._KEY_KICK,
            self._KEY_CHIP,
            self._KEY_DRIBBLE,
            self._KEY_KICK_POWER_UP,
            self._KEY_KICK_POWER_DOWN,
            self._KEY_DRIBBLER_UP,
            self._KEY_DRIBBLER_DOWN,
        }
        self._pressed_keys: set[int] = set()
        self._active = False

        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        widget_layout = QVBoxLayout()
        widget_layout.setContentsMargins(0, 0, 0, 0)
        widget_layout.addWidget(self.__create_widgets())
        self.setLayout(widget_layout)

    def __create_widgets(self) -> QWidget:
        """Create the widgets that make up the KeyboardControllerWidget UI.

        :return: a QWidget showing the key bindings legend and a capture hint
        """
        legend_label = QLabel(self._LEGEND)
        legend_label.setAlignment(Qt.AlignmentFlag.AlignLeft)

        hint_label = QLabel(
            "While keyboard input is selected, key presses are captured for "
            "robot control. Select a different input source to type elsewhere."
        )
        hint_label.setWordWrap(True)
        hint_label.setStyleSheet("color: grey; font-style: italic")

        layout = QVBoxLayout()
        layout.addWidget(legend_label)
        layout.addWidget(hint_label)

        panel = QWidget()
        panel.setLayout(layout)
        return panel

    def set_active(self, active: bool) -> None:
        """Activate or deactivate keyboard control.

        While active and visible, the widget grabs the keyboard so the mapped
        keys are captured regardless of which widget has focus.

        :param active: whether keyboard input should be the active source
        """
        self._active = active
        if active and self.isVisible():
            self.__grab()
        else:
            self.__release()

    def poll(self) -> None:
        """Read the currently pressed keys and update the manual control outputs."""
        # If our window lost focus while keys were held, we will have missed the
        # corresponding release events, so stop the robot rather than driving it
        # on stale key state.
        if not self.isActiveWindow():
            self._pressed_keys.clear()

        self._apply_inputs(
            move_x=self.__axis(self._KEY_BACKWARD, self._KEY_FORWARD),
            move_y=self.__axis(self._KEY_RIGHT, self._KEY_LEFT),
            move_rot=self.__axis(self._KEY_ROTATE_RIGHT, self._KEY_ROTATE_LEFT),
            slow_mode=self.__is_pressed(self._KEY_SLOW),
            dribbler_on=self.__is_pressed(self._KEY_DRIBBLE),
            d_pad_axis_x=self.__axis(
                self._KEY_KICK_POWER_UP, self._KEY_KICK_POWER_DOWN
            ),
            d_pad_axis_y=self.__axis(self._KEY_DRIBBLER_DOWN, self._KEY_DRIBBLER_UP),
            kick=self.__is_pressed(self._KEY_KICK),
            chip=self.__is_pressed(self._KEY_CHIP),
        )

    def keyPressEvent(self, event: QtGui.QKeyEvent) -> None:
        """Record a mapped key being pressed.

        :param event: the key press event
        """
        if not event.isAutoRepeat() and event.key() in self._tracked_keys:
            self._pressed_keys.add(event.key())

    def keyReleaseEvent(self, event: QtGui.QKeyEvent) -> None:
        """Record a mapped key being released.

        :param event: the key release event
        """
        if not event.isAutoRepeat():
            self._pressed_keys.discard(event.key())

    def showEvent(self, event: QtGui.QShowEvent) -> None:
        """Re-grab the keyboard if keyboard input is active and we become visible."""
        super().showEvent(event)
        if self._active:
            self.__grab()

    def hideEvent(self, event: QtGui.QHideEvent) -> None:
        """Release the keyboard when hidden (e.g. switching away from diagnostics)."""
        super().hideEvent(event)
        self.__release()

    def __grab(self) -> None:
        """Grab the keyboard so mapped keys are captured regardless of focus."""
        self.setFocus()
        self.grabKeyboard()

    def __release(self) -> None:
        """Release the keyboard and forget any keys that were held."""
        self.releaseKeyboard()
        self._pressed_keys.clear()

    def __is_pressed(self, key: int) -> bool:
        """:return: whether the given key is currently held down"""
        return key in self._pressed_keys

    def __axis(self, positive_key: int, negative_key: int) -> int:
        """Combine two keys into a -1 / 0 / +1 axis value.

        :param positive_key: the key contributing +1 while held
        :param negative_key: the key contributing -1 while held
        :return: +1, -1 or 0 depending on which of the keys are held
        """
        return self.__is_pressed(positive_key) - self.__is_pressed(negative_key)
