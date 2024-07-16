import time
import logging
import pathlib

import pyqtgraph as pg
from dataclasses import dataclass
from proto.q_learning_pb2 import *
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.constants import SAVED_Q_FUNCTION_WEIGHTS_PATH
from software.thunderscope.common.bar_graph_widget import BarGraphWidget
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *


class QLearningInfoWidget(QWidget):
    """Shows current information about Q-learning algorithms in our AI"""

    ITEM_SIZE_HINT_WIDTH_EXPANSION = 10
    """Empirically makes even bolded items fit within columns"""

    TABLE_DEFAULT_COLUMN_WIDTH = 200
    """Default width of columns in the weights table"""

    SELECTED_ACTION_BAR_COLOUR = QtGui.QColor(68, 138, 255)
    """Bar colour of the selected action in the ActionSelectionStrategyInfo bar graph"""

    @dataclass
    class MdpInfo:
        """Stores information about Q-learning algorithms for a particular 
        Markov decision process (MDP)
        """

        linear_q_func_info: LinearQFunctionInfo = None
        action_selection_strategy_info: ActionSelectionStrategyInfo = None

    def __init__(self, buffer_size: int = 1) -> None:
        """Creates a QLearningInfoWidget

        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary
        """
        QWidget.__init__(self)

        self.linear_q_func_info_buffer = ThreadSafeBuffer(
            buffer_size, LinearQFunctionInfo, False
        )
        self.action_selection_strategy_info_buffer = ThreadSafeBuffer(
            buffer_size, ActionSelectionStrategyInfo, False
        )

        self.mdp_info_dict = {}

        self.linear_q_func_weights_table = QTableWidget(0, 0)
        self.linear_q_func_weights_table.horizontalHeader().setDefaultSectionSize(
            QLearningInfoWidget.TABLE_DEFAULT_COLUMN_WIDTH
        )

        self.mdp_combo_box = QComboBox()
        self.mdp_combo_box.currentTextChanged.connect(
            self.__display_linear_q_function_weights
        )
        self.mdp_combo_box.currentTextChanged.connect(
            self.__display_action_selection_strategy_info
        )

        self.save_weights_button = QPushButton("Save Weights")
        self.save_weights_button.clicked.connect(self.__save_linear_q_function_weights)

        self.action_bar_graph = BarGraphWidget()

        vertical_layout = QVBoxLayout()
        vertical_layout.addWidget(self.mdp_combo_box)
        vertical_layout.addWidget(self.linear_q_func_weights_table)
        vertical_layout.addWidget(self.save_weights_button)
        self.weights_widget = QWidget()
        self.weights_widget.setLayout(vertical_layout)

        self.splitter = QSplitter()
        self.splitter.addWidget(self.weights_widget)
        self.splitter.addWidget(self.action_bar_graph)
        self.splitter.setStretchFactor(0, 3)
        self.splitter.setStretchFactor(1, 1)

        self.layout = QVBoxLayout()
        self.layout.addWidget(self.splitter)
        self.setLayout(self.layout)

    def refresh(self) -> None:
        """Update the Q-learning info widget with new information"""

        linear_q_func_info = self.linear_q_func_info_buffer.get(
            block=False, return_cached=False
        )

        # Updating QTableWidget could be expensive, so we only update if there is new data
        if linear_q_func_info is not None:
            mdp_name = linear_q_func_info.mdp_name

            # Add this MDP to combo box if it is new
            if mdp_name not in self.mdp_info_dict:
                self.mdp_combo_box.addItem(mdp_name)
                self.mdp_info_dict[mdp_name] = QLearningInfoWidget.MdpInfo()

            self.mdp_info_dict[mdp_name].linear_q_func_info = linear_q_func_info

            # Update QTableWidget if this MDP is currently selected in combo box
            if self.mdp_combo_box.currentText() == mdp_name:
                self.__display_linear_q_function_weights(mdp_name)

        action_selection_strategy_info = self.action_selection_strategy_info_buffer.get(
            block=False, return_cached=False
        )

        # Updating BarGraphWidget could be expensive, so we only update if there is new data
        if action_selection_strategy_info is not None:
            mdp_name = action_selection_strategy_info.mdp_name

            # Add this MDP to combo box if it is new
            if mdp_name not in self.mdp_info_dict:
                self.mdp_combo_box.addItem(mdp_name)
                self.mdp_info_dict[mdp_name] = QLearningInfoWidget.MdpInfo()

            self.mdp_info_dict[
                mdp_name
            ].action_selection_strategy_info = action_selection_strategy_info

            # Update BarGraphWidget if this MDP is currently selected in combo box
            if self.mdp_combo_box.currentText() == mdp_name:
                self.__display_action_selection_strategy_info(mdp_name)

    def __display_linear_q_function_weights(self, mdp_name: str) -> None:
        """Update the QTableWidget with the weights of the LinearQFunction
        associated with the given MDP

        :param mdp_name: the name identifying the MDP
        """
        # Check if we have a LinearQFunctionInfo for the given MDP
        if (mdp_name not in self.mdp_info_dict) or (
            not self.mdp_info_dict[mdp_name].linear_q_func_info
        ):
            return

        linear_q_func_info = self.mdp_info_dict[mdp_name].linear_q_func_info

        num_features = linear_q_func_info.num_features
        num_actions = linear_q_func_info.num_actions

        # Setting table size dynamically
        self.linear_q_func_weights_table.setColumnCount(num_features)
        self.linear_q_func_weights_table.setRowCount(num_actions)

        # Set table data with weights
        for row in range(num_actions):
            for col in range(num_features):
                weight = str(linear_q_func_info.weights[row * num_features + col])
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

        # Set row and column headers
        self.linear_q_func_weights_table.setHorizontalHeaderLabels(
            linear_q_func_info.feature_names
        )
        self.linear_q_func_weights_table.setVerticalHeaderLabels(
            linear_q_func_info.action_names
        )

    def __display_action_selection_strategy_info(self, mdp_name: str) -> None:
        """Update the BarGraphWidget with data from the most recent 
        ActionSelectionStrategyInfo for the given MDP
        
        :param mdp_name: the name identifying the MDP
        """
        # Check if we have an ActionSelectionStrategyInfo for the given MDP
        if (mdp_name not in self.mdp_info_dict) or (
            not self.mdp_info_dict[mdp_name].action_selection_strategy_info
        ):
            return

        action_selection_strategy_info = self.mdp_info_dict[
            mdp_name
        ].action_selection_strategy_info

        bar_graph_data = {}
        bar_graph_colours = {}
        for action in action_selection_strategy_info.actions:
            bar_graph_data[action.name] = action.value
            if action.selected:
                bar_graph_colours[
                    action.name
                ] = QLearningInfoWidget.SELECTED_ACTION_BAR_COLOUR

        self.action_bar_graph.set_data(bar_graph_data, colours=bar_graph_colours)

        self.action_bar_graph.set_vertical_axis_label(
            action_selection_strategy_info.action_value_description
        )

    def __save_linear_q_function_weights(self) -> None:
        """Open a file dialog to save the weights of the currently selected 
        LinearQFunction to a CSV file
        """
        mdp_name = self.mdp_combo_box.currentText()

        if (mdp_name not in self.mdp_info_dict) or (
            not self.mdp_info_dict[mdp_name].linear_q_func_info
        ):
            return

        linear_q_func_info = self.mdp_info_dict[mdp_name].linear_q_func_info

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
            f"{SAVED_Q_FUNCTION_WEIGHTS_PATH}/{mdp_name}_weights_{int(time.time())}.csv",
            options=QFileDialog.Option.DontUseNativeDialog,
        )

        # Save weights as CSV file
        if fileName:
            weights = ",".join(map(str, linear_q_func_info.weights))
            with open(fileName, "w") as f:
                f.write(weights)
