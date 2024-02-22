import time
from PyQt6.QtWidgets import *
from software.thunderscope.common.frametime_counter import FrameTimeCounter
from software.py_constants import MILLISECONDS_PER_SECOND

class FrameTimeWidget(QWidget):
    """
    Display the fps and frametime of thunderscope. 
    This is measured in two different places, the buffer callback in GLWidget, and
    the refresh function in tab.

    Also, this widget update every 0.5 seconds
    """
    def __init__(self, buffer_counter:FrameTimeCounter, refresh_counter: FrameTimeCounter, update_delta=0.5):
        """
        Initialize FrameTimeWidget 

        :buffer_counter: a frametime counter at the GLWidget Widget
        :refresh_counter: a frametime counter at the refresh function
        """
        super().__init__()
        self.buffer_counter = buffer_counter
        self.refresh_counter = refresh_counter

        self.fps_label = QLabel("some string to be show") 
        self.fps_label.setText("some fps: ")
        self.vertical_layout = QVBoxLayout()
        self.vertical_layout.addWidget(self.fps_label)

        self.setLayout(self.vertical_layout)

        self.last_update_time = time.time()
        self.update_delta = update_delta # updating every 0.5 seconds

    def refresh(self):
        """
        Updating the fps based on the frametime_counter
        """
        # updating every self.update_delta time, which is likely 0.5 seconds
        if time.time() - self.last_update_time < self.update_delta:
            return

        buffer_frametime = self.buffer_counter.get_last_frametime() * MILLISECONDS_PER_SECOND
        buffer_frametime_average_last_30  = self.buffer_counter.get_average_last_30() * MILLISECONDS_PER_SECOND
        buffer_frametime_average_all = self.buffer_counter.get_average_frametime() * MILLISECONDS_PER_SECOND
        buffer_fps =  1/(buffer_frametime/MILLISECONDS_PER_SECOND)
        buffer_average_last_30_fps = 1/(buffer_frametime_average_last_30/MILLISECONDS_PER_SECOND)
        buffer_fps_all = 1/(buffer_frametime_average_all/MILLISECONDS_PER_SECOND)

        refresh_func_frametime = self.refresh_counter.get_last_frametime() * MILLISECONDS_PER_SECOND
        refresh_func_average_last_30  = self.refresh_counter.get_average_last_30() * MILLISECONDS_PER_SECOND
        refresh_func_frametime_average_all = self.refresh_counter.get_last_frametime() * MILLISECONDS_PER_SECOND
        refresh_func_fps =  1/(refresh_func_frametime/MILLISECONDS_PER_SECOND)
        refresh_func_average_last_30_fps = 1/(refresh_func_average_last_30/MILLISECONDS_PER_SECOND)
        refresh_func_fps_all = 1/(refresh_func_frametime_average_all/MILLISECONDS_PER_SECOND)

        display_text = f"""
        Bufferswap time:
        frametime: {buffer_frametime} 
        fps: {buffer_fps:3f}\n
        last 30: {buffer_frametime_average_last_30} 
        last 30_fps: {buffer_average_last_30_fps}\n
        frametime all: {buffer_frametime_average_all}
        fps_all: {buffer_fps_all}\n  
        
        Refresh Function:
        frametime: {refresh_func_frametime} 
        fps: {refresh_func_fps:3f}\n
        last 30: {refresh_func_average_last_30} 
        last 30_fps: {refresh_func_average_last_30_fps}\n
        frametime all: {refresh_func_frametime_average_all}
        fps_all: {refresh_func_fps_all}\n  

        """

        self.fps_label.setText(display_text)
        self.last_update_time = time.time()
