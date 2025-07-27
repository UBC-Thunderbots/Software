from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt.QtCore import *
from software.py_constants import *
from proto.estop_state_pb2 import EstopState
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class EstopView(QLabel):
    """Class to show whether the estop is playing or stopped"""

    def __init__(self) -> None:
        super().__init__()

        self.estop_state_buffer = ThreadSafeBuffer(1, EstopState)

        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setText("Disconnected")
        self.setStyleSheet("background-color: blue")

    def refresh(self) -> None:
        """Refresh the label"""
        estop_state = self.estop_state_buffer.get(block=False)

        if estop_state.is_playing:
            self.setText("Playing")
            self.setStyleSheet("background-color: green")
        else:
            self.setText("Stopped")
            self.setStyleSheet("background-color: red")
