import pyqtgraph as pg
import pyqtgraph.console as pg_console
from proto.play_info_msg_pb2 import PlayInfo
from software.networking.unix.threaded_unix_listener import ThreadedUnixListener
import software.thunderscope.constants as constants
from google.protobuf.json_format import MessageToDict
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from proto.import_all_protos import *
from software.thunderscope.common.common_widgets import set_table_data

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class PlayInfoWidget(QWidget):

    NUM_ROWS = 6
    NUM_COLS = 4

    # empirically makes even bolded items fit within columns
    HEADER_SIZE_HINT_WIDTH_EXPANSION = 12
    ITEM_SIZE_HINT_WIDTH_EXPANSION = 10

    def __init__(self, minimum_column_width: int = 200, buffer_size: int = 5) -> None:
        """Shows the current play information including tactic and FSM state

        :param minimum_column_width: minimum width of columns
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        QWidget.__init__(self)

        self.play_table = QTableWidget(PlayInfoWidget.NUM_ROWS, PlayInfoWidget.NUM_COLS)

        self.playinfo_buffer = ThreadSafeBuffer(buffer_size, PlayInfo, False)
        self.play_table.verticalHeader().setVisible(False)

        self.vertical_layout = QVBoxLayout()
        self.vertical_layout.addWidget(self.play_table)
        self.setLayout(self.vertical_layout)

    def refresh(self) -> None:
        """Update the play info widget with new play information
        """
        playinfo = self.playinfo_buffer.get(block=False)

        play_info_dict = MessageToDict(playinfo)

        robot_ids = []
        tactic_fsm_states = []
        tactic_names = []
        play_name = []

        if "robotTacticAssignment" not in play_info_dict:
            return

        num_rows = max(
            len(play_info_dict["robotTacticAssignment"]),
            len(play_info_dict["play"]["playState"]),
        )

        # setting table size dynamically
        self.play_table.setRowCount(num_rows)

        for state in play_info_dict["play"]["playState"]:
            play_name.append(state)

        for robot_id in sorted(play_info_dict["robotTacticAssignment"]):
            robot_ids.append(robot_id)
            tactic_fsm_states.append(
                play_info_dict["robotTacticAssignment"][robot_id]["tacticFsmState"]
            )
            tactic_names.append(
                play_info_dict["robotTacticAssignment"][robot_id]["tacticName"]
            )

        set_table_data(
            {
                "Play": play_name,
                "Robot ID": robot_ids,
                "Tactic Name": tactic_names,
                "Tactic FSM State": tactic_fsm_states,
            },
            self.play_table,
            PlayInfoWidget.HEADER_SIZE_HINT_WIDTH_EXPANSION,
            PlayInfoWidget.ITEM_SIZE_HINT_WIDTH_EXPANSION,
        )

        self.play_table.resizeColumnsToContents()
        self.play_table.resizeRowsToContents()
