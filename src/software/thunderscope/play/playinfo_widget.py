from proto.play_info_msg_pb2 import PlayInfo
from pyqtgraph.Qt.QtWidgets import *
from proto.import_all_protos import *
from software.thunderscope.common.common_widgets import set_table_data

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from PyQt6.QtCore import PYQT_VERSION_STR, QT_VERSION_STR
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtCore import QUrl

print(f"PyQt6 Version: {PYQT_VERSION_STR}")
print(f"Qt Version (underlying C++ library): {QT_VERSION_STR}")



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
        self.play_transitions = set()

        QWidget.__init__(self)

        self.webEngineView = QWebEngineView()
        self.loadPage()
        
        self.play_table = QTableWidget(PlayInfoWidget.NUM_ROWS, PlayInfoWidget.NUM_COLS)

        self.playinfo_buffer = ThreadSafeBuffer(buffer_size, PlayInfo, False)
        self.play_table.verticalHeader().setVisible(False)
        self.last_playinfo = None

        self.horizontal_layout = QHBoxLayout()
        self.horizontal_layout.addWidget(self.play_table)
        self.horizontal_layout.addWidget(self.webEngineView)
        self.setLayout(self.horizontal_layout)

    def loadPage(self) -> None:
        html = """<html>
        <body>
            <h1>Play tree</h1>

            <div style="width:100%; overflow-x: scroll;">
                <pre class="mermaid" >
            graph TD
                """

        for t in self.play_transitions:
            html += t + "\n";

        html += """
                </pre>
            </div>
            <script type="module">
            import mermaid from "https://cdn.jsdelivr.net/npm/mermaid@11/dist/mermaid.esm.min.mjs";
            mermaid.initialize({ 
            startOnLoad: true,
            flowchart: {
                useMaxWidth: false 
            } });
            </script>
        </body>
        </html>"""
        self.webEngineView.setHtml(html)

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

        last_play_guard = playinfo.play.last_guard.split(" ", 1)
        prev_fsm = ""
        last_play_transition = playinfo.play.last_transition.split(" -> ")
        if len(last_play_transition) == 2:
            last_play_transition.insert(1," -- " + last_play_guard[1] + " --> ")
            msg_len = len(self.play_transitions)
            self.play_transitions.add(''.join(last_play_transition))
            if msg_len != len(self.play_transitions):
                self.loadPage()

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
