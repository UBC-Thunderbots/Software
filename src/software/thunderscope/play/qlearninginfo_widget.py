import time
import logging
import pathlib

import pyqtgraph as pg
import pyqtgraph.console as pg_console
from proto.q_learning_pb2 import LinearQFunctionInfo
from software.networking.unix.threaded_unix_listener import ThreadedUnixListener
from software.thunderscope.constants import SAVED_Q_FUNCTION_WEIGHTS_PATH
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from proto.import_all_protos import *
from google.protobuf.json_format import MessageToDict
from software.thunderscope.common.common_widgets import set_table_data

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class QLearningInfoWidget(QWidget):

    ITEM_SIZE_HINT_WIDTH_EXPANSION = 10
    """Empirically makes even bolded items fit within columns"""

    def __init__(self, buffer_size: int = 1) -> None:
        """Shows current information about Q-learning functions in our AI

        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary
        """
        QWidget.__init__(self)

        self.linear_q_func_info_buffer = ThreadSafeBuffer(
            buffer_size, LinearQFunctionInfo, False
        )
        self.linear_q_func_infos = {}

        self.linear_q_func_weights_table = QTableWidget(0, 0)
        self.linear_q_func_weights_table.horizontalHeader().hide()
        self.linear_q_func_weights_table.verticalHeader().hide()

        self.linear_q_func_combo_box = QComboBox()
        self.linear_q_func_combo_box.currentTextChanged.connect(
            self.__display_linear_q_function_weights
        )

        self.save_linear_q_func_weights_button = QPushButton("Save Weights")
        self.save_linear_q_func_weights_button.clicked.connect(
            self.__save_linear_q_function_weights
        )

        self.vertical_layout = QVBoxLayout()
        self.vertical_layout.addWidget(self.linear_q_func_combo_box)
        self.vertical_layout.addWidget(self.linear_q_func_weights_table)
        self.vertical_layout.addWidget(self.save_linear_q_func_weights_button)
        self.setLayout(self.vertical_layout)

    def refresh(self) -> None:
        """Update the Q-learning info widget with new information"""

        linear_q_func_info = self.linear_q_func_info_buffer.get(
            block=False, return_cached=False
        )

        # Updating QTableWidget could be expensive, so we only update if there is new data
        if linear_q_func_info is None:
            return

        linear_q_func_info_dict = MessageToDict(linear_q_func_info)
        linear_q_func_name = linear_q_func_info_dict["name"]

        # Add this LinearQFunction to combo box if it is new
        if linear_q_func_name not in self.linear_q_func_infos:
            self.linear_q_func_combo_box.addItem(linear_q_func_name)

        self.linear_q_func_infos[linear_q_func_name] = linear_q_func_info_dict

        # Update QTableWidget if this LinearQFunction is currently selected in combo box
        if self.linear_q_func_combo_box.currentText() == linear_q_func_name:
            self.__display_linear_q_function_weights(linear_q_func_name)

    def __display_linear_q_function_weights(self, linear_q_func_name: str) -> None:
        """Update the QTableWidget with the weights of the specified LinearQFunction

        :param linear_q_func_name: the name identifying the LinearQFunction
        """

        if linear_q_func_name not in self.linear_q_func_infos:
            return

        linear_q_func_info = self.linear_q_func_infos[linear_q_func_name]

        num_features = linear_q_func_info["numFeatures"]
        num_actions = linear_q_func_info["numActions"]

        # Setting table size dynamically
        self.linear_q_func_weights_table.setColumnCount(num_features)
        self.linear_q_func_weights_table.setRowCount(num_actions)

        # Set table data with weights
        for row in range(num_actions):
            for col in range(num_features):
                weight = str(linear_q_func_info["weights"][row * num_features + col])
                table_item = QTableWidgetItem(weight)
                table_item.setSizeHint(
                    QtCore.QSize(
                        len(weight)
                        * QLearningInfoWidget.ITEM_SIZE_HINT_WIDTH_EXPANSION,
                        1,
                    )
                )
                table_item.setFlags(
                    table_item.flags() & ~QtCore.Qt.ItemFlag.ItemIsEditable
                )
                self.linear_q_func_weights_table.setItem(row, col, table_item)

        self.linear_q_func_weights_table.resizeColumnsToContents()
        self.linear_q_func_weights_table.resizeRowsToContents()

    def __save_linear_q_function_weights(self) -> None:
        """Open a file dialog to save the weights of the currently selected 
        LinearQFunction to a CSV file
        """
        linear_q_func_name = self.linear_q_func_combo_box.currentText()
        if linear_q_func_name not in self.linear_q_func_infos:
            return
        linear_q_func_info = self.linear_q_func_infos[linear_q_func_name]

        # Create a folder at SAVED_Q_FUNCTION_WEIGHTS_PATH if it doesn't exist
        try:
            pathlib.Path(SAVED_Q_FUNCTION_WEIGHTS_PATH).mkdir(exist_ok=True)
        except FileNotFoundError:
            logging.warning(
                f"Could not create folder at '{SAVED_Q_FUNCTION_WEIGHTS_PATH}'"
            )
            return

        # Open file dialog
        fileName, _ = QFileDialog.getSaveFileName(
            self,
            "Save weights as CSV",
            f"{SAVED_Q_FUNCTION_WEIGHTS_PATH}/{linear_q_func_name}_weights_{int(time.time())}.csv",
            options=QFileDialog.Option.DontUseNativeDialog,
        )

        # Save weights as CSV file
        if fileName:
            weights = ",".join(map(str, linear_q_func_info["weights"]))
            with open(fileName, "w") as f:
                f.write(weights)
