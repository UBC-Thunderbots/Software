import pyqtgraph as pg
import pyqtgraph.console as pg_console
from proto.play_info_msg_pb2 import PlayInfo
from software.networking.threaded_unix_listener import ThreadedUnixListener
import software.thunderscope.constants as constants

from proto.robot_log_msg_pb2 import RobotLog


class playInfoWidget(pg.DataTreeWidget):
    def __init__(self):
        pg.DataTreeWidget.__init__(self)

        self.playinfo_receiver = ThreadedUnixListener(
            constants.UNIX_SOCKET_BASE_PATH + PlayInfo.DESCRIPTOR.full_name, PlayInfo, max_buffer_size=1
      
        )
        self.cached_playinfo = PlayInfo()
        

    def refresh(self):
        """Update the log widget with another log message
        """
        playinfo = self.playinfo_receiver.maybe_pop()

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

