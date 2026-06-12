from abc import ABCMeta

from pyqtgraph.Qt.QtCore import Qt, QObject, QEvent
from pyqtgraph.Qt.QtWidgets import QApplication

from software.thunderscope.robot_diagnostics.controller_base import ControllerBase

# evdev-independent copies of the ecodes values used by HandheldControllerWidget
_ABS_X = 0
_ABS_Y = 1
_ABS_Z = 2
_ABS_RX = 3
_ABS_RZ = 5
_ABS_HAT0X = 16
_ABS_HAT0Y = 17
_BTN_A = 304
_BTN_B = 305

# Maps abs_code -> (negative_key, positive_key).
# A held negative key returns -1.0; a held positive key returns +1.0.
_ABS_KEY_MAP: dict[int, tuple[Qt.Key | None, Qt.Key | None]] = {
    _ABS_Y: (Qt.Key.Key_W, Qt.Key.Key_S),  # forward / back
    _ABS_X: (Qt.Key.Key_A, Qt.Key.Key_D),  # strafe left / right
    _ABS_RX: (Qt.Key.Key_Q, Qt.Key.Key_E),  # rotate CCW / CW
    _ABS_Z: (None, Qt.Key.Key_Shift),  # slowdown (left trigger)
    _ABS_HAT0X: (Qt.Key.Key_Left, Qt.Key.Key_Right),  # step kick power
    _ABS_HAT0Y: (Qt.Key.Key_Up, Qt.Key.Key_Down),  # step dribbler RPM
    _ABS_RZ: (None, Qt.Key.Key_R),  # dribbler hold (right trigger)
}

# Maps key_code (ecodes int) -> Qt key for digital button inputs
_BTN_KEY_MAP: dict[int, Qt.Key] = {
    _BTN_A: Qt.Key.Key_X,  # kick
    _BTN_B: Qt.Key.Key_C,  # chip
}


class _QABCMeta(type(QObject), ABCMeta):
    pass


class KeyboardController(QObject, ControllerBase, metaclass=_QABCMeta):
    """Keyboard input source.

    Installs a QApplication-level event filter so key events are captured
    regardless of which widget currently has focus.
    """

    def __init__(self) -> None:
        super().__init__()
        self._held_keys: set[Qt.Key] = set()
        self._active = True
        QApplication.instance().installEventFilter(self)

    def name(self) -> str:
        return "Keyboard"

    def connected(self) -> bool:
        return self._active

    def key_down(self, key_code: int) -> bool:
        qt_key = _BTN_KEY_MAP.get(key_code)
        if qt_key is None:
            return False
        return qt_key in self._held_keys

    def abs_value(self, abs_code: int) -> float:
        key_pair = _ABS_KEY_MAP.get(abs_code)
        if key_pair is None:
            return 0.0
        neg_key, pos_key = key_pair
        if neg_key is not None and neg_key in self._held_keys:
            return -1.0
        if pos_key is not None and pos_key in self._held_keys:
            return 1.0
        return 0.0

    def close(self) -> None:
        self._active = False
        self._held_keys.clear()
        app = QApplication.instance()
        if app is not None:
            app.removeEventFilter(self)

    def eventFilter(self, obj: QObject, event: QEvent) -> bool:
        if event.type() == QEvent.Type.KeyPress and not event.isAutoRepeat():
            self._held_keys.add(Qt.Key(event.key()))
        elif event.type() == QEvent.Type.KeyRelease and not event.isAutoRepeat():
            self._held_keys.discard(Qt.Key(event.key()))
        return False
