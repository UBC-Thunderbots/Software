import pyqtgraph as pg
from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.Qt.QtWidgets import *
from software.python_constants import *
import software.thunderscope.common_widgets as common_widgets

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class EmergencyStopStatus(QWidget):
