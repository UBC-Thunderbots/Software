from collections import defaultdict
from pyqtgraph.Qt.QtWidgets import *
from proto.import_all_protos import *
from software.py_constants import SECONDS_PER_MICROSECOND, SECONDS_PER_MINUTE
from software.thunderscope.common.common_widgets import set_table_data
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class RefereeInfoWidget(QWidget):
    NUM_ROWS = 13
    NUM_COLS = 3

    # empirically makes even bolded items fit within columns
    HEADER_SIZE_HINT_WIDTH_EXPANSION = 12
    ITEM_SIZE_HINT_WIDTH_EXPANSION = 11

    def __init__(self, buffer_size: int = 1) -> None:
        """Shows the referee information

        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary
        """
        QWidget.__init__(self)

        self.referee_table = QTableWidget(
            RefereeInfoWidget.NUM_ROWS, RefereeInfoWidget.NUM_COLS
        )
        self.referee_info = QLabel()
        self.referee_buffer = ThreadSafeBuffer(buffer_size, Referee, False)
        self.referee_table.verticalHeader().setVisible(False)

        self.vertical_layout = QVBoxLayout()
        self.vertical_layout.addWidget(self.referee_table)
        self.vertical_layout.addWidget(self.referee_info)
        self.setLayout(self.vertical_layout)

        # Team info table data, indexed by [field_name]["blue" | "yellow"]
        self.team_info = defaultdict(dict)

    def refresh(self) -> None:
        """Update the referee info widget with new referee information"""
        referee = self.referee_buffer.get(block=False, return_cached=False)

        # Updating QTableWidget could be expensive, so we only update if there is new data
        if referee is None:
            return

        stage_time_left_s = referee.stage_time_left * SECONDS_PER_MICROSECOND
        self.referee_info.setText(
            f"Packet Timestamp: {round(referee.packet_timestamp * SECONDS_PER_MICROSECOND, 3)}\n"
            + f"Stage Time Left: {int(stage_time_left_s / SECONDS_PER_MINUTE):02d}"
            + f":{int(stage_time_left_s % SECONDS_PER_MINUTE):02d}\n"
            + f"Stage: {Referee.Stage.Name(referee.stage)}\n"
            + f"Command: {Referee.Command.Name(referee.command)}\n"
            + f"Blue Team on Positive Half: {referee.blue_team_on_positive_half}\n"
        )

        for field_descriptor, field_val in referee.blue.ListFields():
            self.team_info[field_descriptor.name]["blue"] = field_val

        for field_descriptor, field_val in referee.yellow.ListFields():
            self.team_info[field_descriptor.name]["yellow"] = field_val

        set_table_data(
            {
                "Team Info": self.team_info.keys(),
                "Blue": [val["blue"] for val in self.team_info.values()],
                "Yellow": [val["yellow"] for val in self.team_info.values()],
            },
            self.referee_table,
            RefereeInfoWidget.HEADER_SIZE_HINT_WIDTH_EXPANSION,
            RefereeInfoWidget.ITEM_SIZE_HINT_WIDTH_EXPANSION,
        )

        self.referee_table.resizeColumnsToContents()
        self.referee_table.resizeRowsToContents()

    def parse_yellow_card_times(self, team_info: TeamInfo) -> str:
        """Parses yellow card times from a TeamInfo Protobuf dict as a string output.

        :param team_info: TeamInfo protobuf dict to parse
        """
        return [
            time * SECONDS_PER_MICROSECOND for time in team_info.yellow_card_times
        ].join(", ")
