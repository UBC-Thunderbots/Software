import pyqtgraph as pg
import pyqtgraph.console as pg_console
from proto.play_info_msg_pb2 import PlayInfo
from software.networking.threaded_unix_listener import ThreadedUnixListener
import software.thunderscope.constants as constants
from google.protobuf.json_format import MessageToDict
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from proto.import_all_protos import *

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class playInfoWidget(QTableWidget):

    # TODO (#2560): set these values dynamically
    NUM_ROWS = 6
    NUM_COLS = 4

    def __init__(self, buffer_size=5):
        """Shows the current play information including tactic and FSM state

        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        QTableWidget.__init__(self, playInfoWidget.NUM_ROWS, playInfoWidget.NUM_COLS)

        self.playinfo_buffer = ThreadSafeBuffer(buffer_size, PlayInfo, False)
        self.verticalHeader().setVisible(False)

    def set_data(self, data):
        """Data to set in the table

        :param data: dict containing {"column_name": [column_items]}

        """
        horizontal_headers = []

        for n, key in enumerate(sorted(data.keys())):
            horizontal_headers.append(key)

            for m, item in enumerate(data[key]):
                newitem = QTableWidgetItem(item)
                self.setItem(m, n, newitem)

        self.setHorizontalHeaderLabels(horizontal_headers)

    def refresh(self):
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

        play_name.append(play_info_dict["play"]["playName"])

        for robot_id in sorted(play_info_dict["robotTacticAssignment"]):
            robot_ids.append(robot_id)
            tactic_fsm_states.append(
                play_info_dict["robotTacticAssignment"][robot_id]["tacticFsmState"]
            )
            tactic_names.append(
                play_info_dict["robotTacticAssignment"][robot_id]["tacticName"]
            )

        self.set_data(
            {
                "Robot ID": robot_ids,
                "Tactic Name": tactic_names,
                "Tactic FSM State": tactic_fsm_states,
                "Play Name": play_name,
            }
        )

        self.resizeColumnsToContents()
        self.resizeRowsToContents()
