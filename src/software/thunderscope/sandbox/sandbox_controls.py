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

        self.is_playing = False
        # this will be set outside the class
        self.toggle_play_state = None

        self.controls_layout = QHBoxLayout()
        self.controls_box, self.controls = common_widgets.create_buttons(
            ["↩", "↪", "⏸"]
        )

        self.undo_button = self.controls[0]
        self.redo_button = self.controls[1]

        self.play_button = self.controls[2]
        self.play_button.clicked.connect(self.toggle_play)

        self.controls_layout.addWidget(self.controls_box)

        self.setLayout(self.controls_layout)

    def toggle_play(self):
        """
        Calls the function to toggle the play state
        """
        self.is_playing = self.toggle_play_state()

    def refresh(self):
        """
        Refreshes the UI based on the current state
        """
        self.play_button.setText("⏸" if self.is_playing else "▶")
