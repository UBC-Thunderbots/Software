import time
from PyQt6.QtWidgets import *
import PyQt6.QtCore
from software.thunderscope.common.frametime_counter import FrameTimeCounter
from software.py_constants import MILLISECONDS_PER_SECOND


class FrameTimeWidget(QWidget):
    """
    Display the fps and frametime of thunderscope. 
    This is measured in two different places, the buffer callback in GLWidget, and
    the refresh function in tab.

    Also, this widget update every 0.5 seconds
    """

    def __init__(
        self,
        buffer_counter: FrameTimeCounter,
        refresh_counter: FrameTimeCounter,
        update_delta_sec=0.5,
    ):
        """
        Initialize FrameTimeWidget 

        :buffer_counter: a frametime counter at the GLWidget Widget
        :refresh_counter: a frametime counter at the refresh function
        """
        super().__init__()
        self.buffer_counter = buffer_counter
        self.refresh_counter = refresh_counter

        self.vertical_layout = QVBoxLayout()

        self.buffertime_table = QTableWidget(3, 2)
        self.buffertime_table.setHorizontalHeaderLabels(["Frametime (ms)", "FPS"])
        self.buffertime_table.setVerticalHeaderLabels(["Recent", "Last 30", "All"])
        self.refresh_function_table = QTableWidget(3, 2)
        self.refresh_function_table.setHorizontalHeaderLabels(["Frametime (ms)", "FPS"])
        self.refresh_function_table.setVerticalHeaderLabels(
            ["Recent", "Last 30", "All"]
        )
        self.buffertime_table.resizeColumnsToContents()
        self.refresh_function_table.resizeColumnsToContents()

        buffertime_title = QLabel("<b> Buffertime FPS <b>")
        buffertime_title.setAlignment(PyQt6.QtCore.Qt.AlignmentFlag.AlignHCenter)
        refresh_function_title = QLabel("<b> Refresh Function FPS <b>")
        refresh_function_title.setAlignment(PyQt6.QtCore.Qt.AlignmentFlag.AlignHCenter)

        self.vertical_layout.addWidget(buffertime_title)
        self.vertical_layout.addWidget(self.buffertime_table)
        self.vertical_layout.addWidget(refresh_function_title)
        self.vertical_layout.addWidget(self.refresh_function_table)

        self.setLayout(self.vertical_layout)
        self.last_update_time = time.time()
        self.update_delta = update_delta_sec  # updating every 0.5 seconds

    def update_table(self, table, row, col, text):
        """
        table: this table that I am going to be updating
        row: the row in which data is going to be updated
        col: the column in which the data is going to be updated
        text: the text in which the data is going to be updated
        Updating what is on the table. This purpose of this section is to reduce Redundancy
        """
        text = QTableWidgetItem(text)
        text.setTextAlignment(PyQt6.QtCore.Qt.AlignmentFlag.AlignHCenter)
        table.setItem(row, col, text)

    def refresh(self):
        """
        Updating the fps based on the frametime_counter
        """
        # updating every self.update_delta time, which is likely 0.5 seconds
        if time.time() - self.last_update_time < self.update_delta:
            return

        buffer_frametime = (
            self.buffer_counter.get_last_frametime() * MILLISECONDS_PER_SECOND
        )
        buffer_frametime_average_last_30 = (
            self.buffer_counter.get_average_last_30() * MILLISECONDS_PER_SECOND
        )
        buffer_frametime_average_all = (
            self.buffer_counter.get_average_frametime() * MILLISECONDS_PER_SECOND
        )
        buffer_fps = 1 / (buffer_frametime / MILLISECONDS_PER_SECOND)
        buffer_average_last_30_fps = 1 / (
            buffer_frametime_average_last_30 / MILLISECONDS_PER_SECOND
        )
        buffer_fps_all = 1 / (buffer_frametime_average_all / MILLISECONDS_PER_SECOND)

        refresh_func_frametime = (
            self.refresh_counter.get_last_frametime() * MILLISECONDS_PER_SECOND
        )
        refresh_func_average_last_30 = (
            self.refresh_counter.get_average_last_30() * MILLISECONDS_PER_SECOND
        )
        refresh_func_frametime_average_all = (
            self.refresh_counter.get_last_frametime() * MILLISECONDS_PER_SECOND
        )
        refresh_func_fps = 1 / (refresh_func_frametime / MILLISECONDS_PER_SECOND)
        refresh_func_average_last_30_fps = 1 / (
            refresh_func_average_last_30 / MILLISECONDS_PER_SECOND
        )
        refresh_func_fps_all = 1 / (
            refresh_func_frametime_average_all / MILLISECONDS_PER_SECOND
        )

        # update table for buffertime
        self.update_table(self.buffertime_table, 0, 0, f"{buffer_frametime:.1f}")
        self.update_table(self.buffertime_table, 0, 1, f"{buffer_fps:.1f}")
        self.update_table(
            self.buffertime_table, 1, 0, f"{buffer_frametime_average_last_30:.1f}"
        )
        self.update_table(
            self.buffertime_table, 1, 1, f"{buffer_average_last_30_fps:.1f}"
        )
        self.update_table(
            self.buffertime_table, 2, 0, f"{buffer_frametime_average_all:.1f}"
        )
        self.update_table(self.buffertime_table, 2, 1, f"{buffer_fps_all:.1f}")

        # update table for refresh function
        self.update_table(
            self.refresh_function_table, 0, 0, f"{refresh_func_frametime:.1f}"
        )
        self.update_table(self.refresh_function_table, 0, 1, f"{refresh_func_fps:.1f}")
        self.update_table(
            self.refresh_function_table,
            1,
            0,
            f"{refresh_func_frametime_average_all:.1f}",
        )
        self.update_table(
            self.refresh_function_table, 1, 1, f"{refresh_func_average_last_30_fps:.1f}"
        )
        self.update_table(
            self.refresh_function_table,
            2,
            0,
            f"{refresh_func_frametime_average_all:.1f}",
        )
        self.update_table(
            self.refresh_function_table, 2, 1, f"{refresh_func_fps_all:.1f}"
        )

        self.last_update_time = time.time()
