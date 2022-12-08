import pyqtgraph as pg
import pyqtgraph.console as pg_console
from proto.play_info_msg_pb2 import PlayInfo
from software.networking.threaded_unix_listener import ThreadedUnixListener
import software.thunderscope.constants as constants
from google.protobuf.json_format import MessageToDict
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from proto.import_all_protos import *
from software.py_constants import SECONDS_PER_MICROSECOND

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class refereeInfoWidget(QWidget):

    NUM_ROWS = 13
    NUM_COLS = 3

    def __init__(self, buffer_size=5):
        """Shows the current play information including tactic and FSM state

        :param minimum_column_width: minimum width of columns
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        QWidget.__init__(self)

        self.referee_table = QTableWidget(
            refereeInfoWidget.NUM_ROWS, refereeInfoWidget.NUM_COLS
        )
        self.referee_info = QLabel()
        self.referee_buffer = ThreadSafeBuffer(buffer_size, Referee, False)

        self.vertical_layout = QVBoxLayout()
        self.vertical_layout.addWidget(self.referee_table)
        self.vertical_layout.addWidget(self.referee_info)
        self.setLayout(self.vertical_layout)

    def set_data(self, data):
        """Data to set in the table

        :param data: dict containing {"column_name": [column_items]}

        """
        horizontal_headers = []

        # empirically makes even bolded items fit within columns
        HEADER_SIZE_HINT_WIDTH_EXPANSION = 12
        ITEM_SIZE_HINT_WIDTH_EXPANSION = 11

        for n, key in enumerate(data.keys()):
            horizontal_headers.append(key)

            for m, item in enumerate(data[key]):
                str_item = str(item)
                newitem = QTableWidgetItem(str_item)
                newitem.setSizeHint(
                    QtCore.QSize(
                        max(
                            len(key) * HEADER_SIZE_HINT_WIDTH_EXPANSION,
                            len(str_item) * ITEM_SIZE_HINT_WIDTH_EXPANSION,
                        ),
                        1,
                    )
                )
                self.referee_table.setItem(m, n, newitem)

        self.referee_table.setHorizontalHeaderLabels(horizontal_headers)

    def refresh(self):
        """Update the play info widget with new play information
        """
        referee = self.referee_buffer.get(block=False)
        referee_msg_dict = MessageToDict(referee)

        if referee_msg_dict:
            p = (
                f"Packet Timestamp: {round(float(referee_msg_dict['packetTimestamp'])*SECONDS_PER_MICROSECOND, 3)}\n"
                + f"Stage: {referee_msg_dict['stageTimeLeft']*SECONDS_PER_MICROSECOND} {referee_msg_dict['stage']}\n"
                + "Command: "
                + referee_msg_dict["command"]
                + "\n"
                + f"Blue Team on Positive Half: {referee_msg_dict['blueTeamOnPositiveHalf']}\n"
            )
            self.referee_info.setText(p)

        team_info = []
        blue = []
        yellow = []

        num_rows = len(referee_msg_dict["blue"])

        # setting table size dynamically
        self.referee_table.setRowCount(num_rows)

        for team_info_name in referee_msg_dict["blue"]:
            if team_info_name == "timeouts":
                team_info.append("remainingTimeouts")
            elif team_info_name == "goalkeeper":
                team_info.append("goalkeeperID")
            else:
                team_info.append(team_info_name)

        for info in team_info:
            if info == "yellowCardTimes":
                text = ""
                for time in referee_msg_dict["blue"]["yellowCardTimes"]:
                    formatted_time = int(time * SECONDS_PER_MICROSECOND)
                    text = text + str(formatted_time) + "\n"
                blue.append(text)
                text = ""
                for time in referee_msg_dict["yellow"]["yellowCardTimes"]:
                    formatted_time = int(time * SECONDS_PER_MICROSECOND)
                    text = text + str(formatted_time) + "\n"
                yellow.append(text)
            elif info == "remainingTimeouts":
                blue.append(referee_msg_dict["blue"]["timeouts"])
                yellow.append(referee_msg_dict["yellow"]["timeouts"])
            elif info == "goalkeeperID":
                blue.append(referee_msg_dict["blue"]["goalkeeper"])
                yellow.append(referee_msg_dict["yellow"]["goalkeeper"])
            else:
                blue.append(referee_msg_dict["blue"][info])
                yellow.append(referee_msg_dict["yellow"][info])

        self.set_data(
            {"Team Info": team_info, "Blue": blue, "Yellow": yellow,}
        )

        self.referee_table.resizeColumnsToContents()
        self.referee_table.resizeRowsToContents()
