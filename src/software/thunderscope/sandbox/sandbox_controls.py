from pyqtgraph.Qt.QtWidgets import *
import software.thunderscope.common.common_widgets as common_widgets


class SandboxControls(QWidget):
    """
    Controls for Sandbox Mode
    Allows playing / pausing the simulation
    And undoing / redoing changes to the robots
    """

    def __init__(self):
        super().__init__()

        self.is_playing = True
        # this will be set outside the class
        self.toggle_play_state = None

        self.controls_layout = QHBoxLayout()
        self.controls_layout.setContentsMargins(0, 0, 0, 0)
        self.controls_box, self.controls = common_widgets.create_buttons(
            ["↩", "↪", "⏸", "Export"]
        )

        self.undo_button = self.controls[0]
        self.redo_button = self.controls[1]

        self.play_button = self.controls[2]
        self.play_button.clicked.connect(self.__toggle_play)

        self.export_button = self.controls[3]

        self.controls_layout.addWidget(self.controls_box)

        self.setLayout(self.controls_layout)

    def __toggle_play(self):
        """
        Calls the function to toggle the play state
        """
        self.is_playing = self.toggle_play_state()

    def refresh(self):
        """
        Refreshes the UI based on the current state
        """
        self.play_button.setText("⏸" if self.is_playing else "▶")
