from collections import defaultdict

import proto.import_all_protos as protos
from pyqtgraph.Qt import QtWidgets
from software.py_constants import SECONDS_PER_MICROSECOND, SECONDS_PER_MINUTE
from software.thunderscope.common.common_widgets import set_table_data
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class RefereeInfoWidget(QtWidgets.QWidget):
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
        QtWidgets.QWidget.__init__(self)

        self.referee_table = QtWidgets.QTableWidget(
            RefereeInfoWidget.NUM_ROWS, RefereeInfoWidget.NUM_COLS
        )
        self.referee_info = QtWidgets.QLabel()
        self.referee_buffer = ThreadSafeBuffer(buffer_size, protos.Referee, False)
        self.referee_table.verticalHeader().setVisible(False)

        self.vertical_layout = QtWidgets.QVBoxLayout()
        self.vertical_layout.addWidget(self.referee_table)
        self.vertical_layout.addWidget(self.referee_info)
        self.setLayout(self.vertical_layout)

        # Team info table data, indexed by [field_name]["blue" | "yellow"]
        self.team_info = defaultdict(dict)

    @staticmethod
    def _format_field(team_info, field_name: str) -> str:
        """Return a display string for one TeamInfo field.

        Works even if the field is unset or an empty list, so both
        teams always have a value for every row.

        :param team_info: the TeamInfo proto for one team
        :param field_name: the name of the field to format
        :return: the value as a string, or "-" if the field has no value
        """
        value = getattr(team_info, field_name)
        field = team_info.DESCRIPTOR.fields_by_name[field_name]

        if field.is_repeated:
            if field_name == "yellow_card_times":
                return ", ".join(f"{t / 1_000_000:.1f}s" for t in value) or "-"
            return ", ".join(str(v) for v in value) or "-"

        return str(value) if team_info.HasField(field_name) else "-"

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
            + f"Stage: {protos.Referee.Stage.Name(referee.stage)}\n"
            + f"Command: {protos.Referee.Command.Name(referee.command)}\n"
            + f"Blue Team on Positive Half: {referee.blue_team_on_positive_half}\n"
        )

        # Use every TeamInfo field (not just populated ones) so both teams
        # always have a value for each row
        field_names = [f.name for f in protos.Referee.TeamInfo.DESCRIPTOR.fields]

        self.referee_table.setRowCount(len(field_names))
        set_table_data(
            {
                "Team Info": field_names,
                "Blue": [self._format_field(referee.blue, f) for f in field_names],
                "Yellow": [self._format_field(referee.yellow, f) for f in field_names],
            },
            self.referee_table,
            RefereeInfoWidget.HEADER_SIZE_HINT_WIDTH_EXPANSION,
            RefereeInfoWidget.ITEM_SIZE_HINT_WIDTH_EXPANSION,
        )

        self.referee_table.resizeColumnsToContents()
        self.referee_table.resizeRowsToContents()

    def parse_yellow_card_times(self, team_info: protos.TeamInfo) -> str:
        """Parses yellow card times from a TeamInfo Protobuf dict as a string output.

        :param team_info: TeamInfo protobuf dict to parse
        """
        return [
            time * SECONDS_PER_MICROSECOND for time in team_info.yellow_card_times
        ].join(", ")
