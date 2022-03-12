import pyqtgraph as pg
import pyqtgraph.console as pg_console
from proto.play_info_msg_pb2 import PlayInfo
from software.networking.threaded_unix_listener import ThreadedUnixListener
import software.thunderscope.constants as constants
import queue

from proto.robot_log_msg_pb2 import RobotLog


class playInfoWidget(pg.DataTreeWidget):
    def __init__(self, buffer_size=10):
        pg.DataTreeWidget.__init__(self)

        self.log_buffer = queue.Queue(buffer_size)
        
    def refresh(self):
        """Update the play info widget with new robot information
        """
        try:
            playinfo = self.log_buffer.get_nowait()
        except queue.Empty as empty:
            return

        d = {
            'play name': playinfo.play.play_name,
            'robot 0': {'tactic name' : playinfo.robot_tactic_assignment[0].tactic_name,
                        'tactic fsm state' : playinfo.robot_tactic_assignment[0].tactic_fsm_state},
            'robot 1': {'tactic name' : playinfo.robot_tactic_assignment[1].tactic_name,
                        'tactic fsm state' : playinfo.robot_tactic_assignment[1].tactic_fsm_state},
            'robot 2': {'tactic name' : playinfo.robot_tactic_assignment[2].tactic_name,
                        'tactic fsm state' : playinfo.robot_tactic_assignment[2].tactic_fsm_state},
            'robot 3': {'tactic name' : playinfo.robot_tactic_assignment[3].tactic_name,
                        'tactic fsm state' : playinfo.robot_tactic_assignment[3].tactic_fsm_state},
            'robot 4': {'tactic name' : playinfo.robot_tactic_assignment[4].tactic_name,
                        'tactic fsm state' : playinfo.robot_tactic_assignment[4].tactic_fsm_state},
            'robot 5': {'tactic name' : playinfo.robot_tactic_assignment[5].tactic_name,
                        'tactic fsm state' : playinfo.robot_tactic_assignment[5].tactic_fsm_state}           
        }

        self.setData(d)

