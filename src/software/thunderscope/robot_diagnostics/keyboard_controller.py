from abc import ABCMeta

from pyqtgraph.Qt.QtCore import Qt, QObject, QEvent
from pyqtgraph.Qt.QtWidgets import QApplication

from software.thunderscope.constants import DiagnosticsConstants
from software.thunderscope.robot_diagnostics.controller_base import ControllerBase


class _QABCMeta(type(QObject), ABCMeta):
    pass


class KeyboardController(QObject, ControllerBase, metaclass=_QABCMeta):
    """Keyboard input source.

    Installs a QApplication-level event filter so key events are captured
    regardless of which widget currently has focus.

    Analog axes (move velocity, speed factor, dribbler hold) are driven by
    held keys. Stepped inputs (kick/chip power, dribbler RPM) and one-shot
    actions (kick, chip) are set as pending flags in the event filter and
    consumed on the first read.
    """

    def __init__(self) -> None:
        super().__init__()
        self._held_keys: set[Qt.Key] = set()
        self._active = True
        self._pending_kick_power_step = 0
        self._pending_dribbler_step = 0
        self._pending_kick = False
        self._pending_chip = False
        QApplication.instance().installEventFilter(self)

    def name(self) -> str:
        return "Keyboard"

    def connected(self) -> bool:
        return self._active

    def close(self) -> None:
        self._active = False
        self._held_keys.clear()
        app = QApplication.instance()
        if app is not None:
            app.removeEventFilter(self)

    def get_move_velocity(self) -> tuple[float, float, float]:
        """Return (vx, vy, vrot) from held movement keys.

        :return: (vx, vy, vrot) where positive x = forward (W), positive y = strafe left (A),
                 positive angular = CCW (Q)
        """

        def axis(neg: Qt.Key, pos: Qt.Key) -> float:
            if neg in self._held_keys:
                return -1.0
            if pos in self._held_keys:
                return 1.0
            return 0.0

        return (
            axis(Qt.Key.Key_S, Qt.Key.Key_W),
            axis(Qt.Key.Key_D, Qt.Key.Key_A),
            axis(Qt.Key.Key_E, Qt.Key.Key_Q),
        )

    def get_speed_factor(self) -> float:
        """Return SPEED_SLOWDOWN_FACTOR if Shift is held, otherwise 1.0."""
        return (
            DiagnosticsConstants.SPEED_SLOWDOWN_FACTOR
            if Qt.Key.Key_Shift in self._held_keys
            else 0.75
        )

    def is_dribbler_held(self) -> bool:
        """Return True if R is held."""
        return Qt.Key.Key_R in self._held_keys

    def get_kick_power_step(self) -> int:
        """Consume and return the pending kick/chip power step (-1, 0, or +1).
        Left arrow = -1, Right arrow = +1.
        """
        step = self._pending_kick_power_step
        self._pending_kick_power_step = 0
        return step

    def get_dribbler_step(self) -> int:
        """Consume and return the pending dribbler RPM step (-1, 0, or +1).
        Up arrow = +1 (increase), Down arrow = -1 (decrease).
        """
        step = self._pending_dribbler_step
        self._pending_dribbler_step = 0
        return step

    def is_kick_fired(self) -> bool:
        """Consume and return whether a kick was pending (X key)."""
        fired = self._pending_kick
        self._pending_kick = False
        return fired

    def is_chip_fired(self) -> bool:
        """Consume and return whether a chip was pending (C key)."""
        fired = self._pending_chip
        self._pending_chip = False
        return fired

    def eventFilter(self, obj: QObject, event: QEvent) -> bool:
        if event.type() == QEvent.Type.KeyPress and not event.isAutoRepeat():
            key = Qt.Key(event.key())
            self._held_keys.add(key)
            if key == Qt.Key.Key_Left:
                self._pending_kick_power_step = -1
            elif key == Qt.Key.Key_Right:
                self._pending_kick_power_step = 1
            elif key == Qt.Key.Key_Up:
                self._pending_dribbler_step = 1
            elif key == Qt.Key.Key_Down:
                self._pending_dribbler_step = -1
            elif key == Qt.Key.Key_X:
                self._pending_kick = True
            elif key == Qt.Key.Key_C:
                self._pending_chip = True
        elif event.type() == QEvent.Type.KeyRelease and not event.isAutoRepeat():
            self._held_keys.discard(Qt.Key(event.key()))
        return False
