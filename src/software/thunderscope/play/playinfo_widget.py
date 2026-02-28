from proto.play_info_msg_pb2 import PlayInfo
from pyqtgraph.Qt.QtWidgets import *
import pyqtgraph as pg
from proto.import_all_protos import *
from software.thunderscope.common.common_widgets import set_table_data

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from graphviz import Digraph
import numpy as np
from PyQt6.QtCore import QUrl

class FSMGrapher:
    def __init__(self):
        """Stores the state transition information as a graphviz graph."""
        self.nodes = set()
        self.edges = []
        self.node_map = {}
    
    def update_structure(self, guard: str, src_node: str, dest_node: str):
        """Update the structure of the graph with the new edge"""

        # need to implement information parsing.
        source = src_node.replace("::", "")
        dest = dest_node.replace("::", "")

        edge_entry = (source, dest, guard)
        if edge_entry not in self.edges:
            self.edges.append(edge_entry)

        self.nodes.add(source)
        self.nodes.add(dest)

        dot = Digraph(engine = 'dot')
        dot.attr(rankdir='LR')
        dot.attr(splines='curved')

        for n in self.nodes:
            dot.node(n)

        for src, dst, grd in self.edges:
            dot.edge(src, dst, label = str(grd), labeldistance='2.0')
        
        raw_layout = dot.pipe(format='plain').decode('utf-8')

        coords = {}
        edge_labels = []
        for line in raw_layout.splitlines():
            parts = line.split()
            if parts[0] == 'node':
                node_id = parts[1].strip('"')
                coords[node_id] = (float(parts[2]), float(parts[3]))
            elif parts[0] == 'edge':
                label_idx = next(i for i, s in enumerate(parts) if '"' in s)
                transition_label = (parts[label_idx] + parts[label_idx + 1]).replace('"', '')
                transition_label = transition_label[transition_label.find(':')+2:]
                text, x, y = transition_label, float(parts[label_idx + 2]), float(parts[label_idx + 3])
                edge_labels.append((text, x, y))

        node_names = list(coords.keys())
        pos = np.array([coords[name] for name in node_names])
        adj = np.array([[node_names.index(s), node_names.index(d)] for s, d, _ in self.edges])

        return node_names, pos, adj, edge_labels        


class PlayInfoWidget(QWidget):
    NUM_ROWS = 6
    NUM_COLS = 6

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

        self.graph = pg.GraphItem()
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.addItem(self.graph)
        self.plot_widget.hideAxis('bottom')
        self.plot_widget.hideAxis('left')
        self.plot_widget.setAspectLocked(True)

        self.horizontal_layout = QHBoxLayout()
        self.horizontal_layout.addWidget(self.play_table)
        self.horizontal_layout.addWidget(self.plot_widget)
        self.setLayout(self.horizontal_layout)

        self.fsm_graph = FSMGrapher()
        self.label_items = []

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
        tactic_guards = []
        play_name = []
        tactic_transitions = []

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

        play_name.append(playinfo.play.last_guard)
        play_name.append(playinfo.play.last_transition)

        play_transition = playinfo.play.last_transition.split(" -> ")
        node_names, pos, adj, labels = self.fsm_graph.update_structure(
            playinfo.play.last_guard.split(" ", 1)[1],
            play_transition[0],
            play_transition[1]
        )

        for item in self.label_items:
            self.plot_widget.removeItem(item)
        self.label_items.clear()

        # Add Node Labels (State Names)
        for i, name in enumerate(node_names):
            # Anchor (0.5, 1.2) puts the center bottom of the text just above the node center
            text = pg.TextItem(name, anchor=(0.5, 1.2), color='w')
            text.setPos(pos[i][0], pos[i][1]) # Use pos X and Y
            self.plot_widget.addItem(text)
            self.label_items.append(text)

        for text_val, x, y in labels:
            lbl = pg.TextItem(text_val, color=(200, 200, 0)) # Yellow for guards
            lbl.setPos(x, y)
            self.plot_widget.addItem(lbl)
            self.label_items.append(lbl)

        active_node = play_transition[1].replace("::","")

        brushes = [pg.mkBrush(150, 150, 150) for _ in node_names]
        if active_node in node_names:
            brushes[node_names.index(active_node)] = pg.mkBrush('g')


        self.graph.setData(pos=pos, adj=adj, symbolBrush=brushes, size=20, symbol='o')

        for robot_id in sorted(playinfo.robot_tactic_assignment):
            robot_ids.append(robot_id)
            tactic_fsm_states.append(
                playinfo.robot_tactic_assignment[robot_id].tactic_fsm_state
            )
            tactic_guards.append(playinfo.robot_tactic_assignment[robot_id].last_guard)
            tactic_transitions.append(playinfo.robot_tactic_assignment[robot_id].last_transition)
            tactic_names.append(playinfo.robot_tactic_assignment[robot_id].tactic_name)

        set_table_data(
            {
                "Play": play_name,
                "Robot ID": robot_ids,
                "Tactic Name": tactic_names,
                "Tactic FSM State": tactic_fsm_states,
                "Tactic Guard": tactic_guards,
                "Tactic Transitions" : tactic_transitions,
            },
            self.play_table,
            PlayInfoWidget.HEADER_SIZE_HINT_WIDTH_EXPANSION,
            PlayInfoWidget.ITEM_SIZE_HINT_WIDTH_EXPANSION,
        )

        self.play_table.resizeColumnsToContents()
        self.play_table.resizeRowsToContents()
