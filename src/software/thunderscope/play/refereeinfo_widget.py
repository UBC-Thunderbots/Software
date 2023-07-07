import pyqtgraph as pg
import software.thunderscope.constants as constants
from google.protobuf.json_format import MessageToDict
from pyqtgraph.Qt import QtCore, QtGui
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

    def __init__(self, buffer_size=5):
        """Shows the referee information 

        :param minimum_column_width: minimum width of columns
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

    def refresh(self):
        """Update the referee info widget with new referee information
        """
        referee = self.referee_buffer.get(block=False)
        referee_msg_dict = MessageToDict(referee)

        if not referee_msg_dict:
            return

        p = (
            f"Packet Timestamp: {round(float(referee_msg_dict['packetTimestamp']) * SECONDS_PER_MICROSECOND, 3)}\n"
            + f"Stage Time Left: {int(referee_msg_dict['stageTimeLeft'] * SECONDS_PER_MICROSECOND / SECONDS_PER_MINUTE)}:{int(referee_msg_dict['stageTimeLeft'] * SECONDS_PER_MICROSECOND % SECONDS_PER_MINUTE)}\n"
            + f"Stage: {referee_msg_dict['stage']}\n"
            + "Command: "
            + referee_msg_dict["command"]
            + "\n"
            + f"Blue Team on Positive Half: {referee_msg_dict['blueTeamOnPositiveHalf']}\n"
        )
        self.referee_info.setText(p)

        team_info = []
        blue = []
        yellow = []

        for team_info_name in referee_msg_dict["blue"]:
            if team_info_name == "timeouts":
                team_info.append("remainingTimeouts")
            elif team_info_name == "goalkeeper":
                team_info.append("goalkeeperID")
            else:
                team_info.append(team_info_name)

        for info in team_info:
            if info == "yellowCardTimes":
                pass
                # for time in referee_msg_dict["blue"]["yellowCardTimes"]:
                #     formatted_time = int(time * SECONDS_PER_MICROSECOND)
                #     text = text + str(formatted_time) + ", "
                # blue.append(text)
                # text = ""
                # for time in referee_msg_dict["yellow"]["yellowCardTimes"]:
                #     formatted_time = int(time * SECONDS_PER_MICROSECOND)
                #     text = text + str(formatted_time) + ", "
                # yellow.append(text)
                # text = ""
                # for time in referee_msg_dict["blue"]["yellowCardTimes"]:
                #     formatted_time = int(time * SECONDS_PER_MICROSECOND)
                #     text = text + str(formatted_time) + ", "
                # blue.append(text)
                # text = ""
                # for time in referee_msg_dict["yellow"]["yellowCardTimes"]:
                #     formatted_time = int(time * SECONDS_PER_MICROSECOND)
                #     text = text + str(formatted_time) + ", "
                # yellow.append(text)
            elif info == "remainingTimeouts":
                blue.append(referee_msg_dict["blue"]["timeouts"])
                yellow.append(referee_msg_dict["yellow"]["timeouts"])
            elif info == "goalkeeperID":
                blue.append(referee_msg_dict["blue"]["goalkeeper"])
                yellow.append(referee_msg_dict["yellow"]["goalkeeper"])
            else:
                blue.append(referee_msg_dict["blue"][info])
                yellow.append(referee_msg_dict["yellow"][info])

        set_table_data(
            {"Team Info": team_info, "Blue": blue, "Yellow": yellow,},
            self.referee_table,
            RefereeInfoWidget.HEADER_SIZE_HINT_WIDTH_EXPANSION,
            RefereeInfoWidget.ITEM_SIZE_HINT_WIDTH_EXPANSION,
        )

        self.referee_table.resizeColumnsToContents()
        self.referee_table.resizeRowsToContents()
