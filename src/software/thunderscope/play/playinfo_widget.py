from proto.play_info_msg_pb2 import PlayInfo
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

    def __init__(self, buffer_size: int = 1) -> None:
        """Shows the current play information including tactic and FSM state

        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary
        """
        QWidget.__init__(self)        

        self.play_table = QTableWidget(PlayInfoWidget.NUM_ROWS, PlayInfoWidget.NUM_COLS)

        self.playinfo_buffer = ThreadSafeBuffer(buffer_size, PlayInfo, False)
        self.play_table.verticalHeader().setVisible(False)
        self.last_playinfo = None

        self.vertical_layout = QVBoxLayout()
        self.vertical_layout.addWidget(self.play_table)
        self.vertical_layout.addWidget(self.webEngineView)
        self.setLayout(self.vertical_layout)

    def refresh(self) -> None:
        """Update the play info widget with new play information"""
        playinfo = self.playinfo_buffer.get(block=False, return_cached=False)

        # Updating QTableWidget could be expensive, so we only update if there is new data
        if playinfo is None or playinfo == self.last_playinfo:
            return

        self.last_playinfo = playinfo

        robot_ids = []
        tactic_fsm_states = []
        tactic_names = []
        play_name = []

        if not playinfo.robot_tactic_assignment:
            return

        num_rows = max(
            len(playinfo.robot_tactic_assignment),
            len(playinfo.play.play_state),
        )

        # setting table size dynamically
        self.play_table.setRowCount(num_rows)

        for state in playinfo.play.play_state:
            play_name.append(state)

        for robot_id in sorted(playinfo.robot_tactic_assignment):
            robot_ids.append(robot_id)
            tactic_fsm_states.append(
                playinfo.robot_tactic_assignment[robot_id].tactic_fsm_state
            )

            tactic_names.append(playinfo.robot_tactic_assignment[robot_id].tactic_name)
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
