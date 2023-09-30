from pyqtgraph.Qt.QtWidgets import *
import software.thunderscope.common.common_widgets as common_widgets

class SandboxControls(QWidget):

    def __init__(self):
        super().__init__()

        self.is_playing = False
        self.toggle_play_state = None

        self.controls_layout = QHBoxLayout()
        self.controls_box, self.controls = common_widgets.create_buttons(
           [
               "Pause"
           ]
        )

        self.play_button = self.controls[0]
        self.play_button.clicked.connect(self.toggle_play)

        self.controls_layout.addWidget(self.controls_box)

        self.setLayout(self.controls_layout)

    def toggle_play(self):
        print('HERE')
        self.is_playing = self.toggle_play_state()

    def refresh(self):
        self.play_button.setText("Pause" if self.is_playing else "Play")