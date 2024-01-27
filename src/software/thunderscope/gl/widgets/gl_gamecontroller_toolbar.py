from software.thunderscope.gl.widgets.gl_toolbar import GLToolbar
from pyqtgraph.Qt.QtWidgets import *
from software.thunderscope.binary_context_managers.game_controller import Gamecontroller
from proto.ssl_gc_common_pb2 import Team
from proto.import_all_protos import *
from proto.ssl_gc_common_pb2 import Team


class GLGamecontrollerToolbar(GLToolbar):
    def __init__(self, gamecontroller: Gamecontroller, friendly_color_yellow: bool):
        super(GLGamecontrollerToolbar, self).__init__()

        self.gamecontroller = gamecontroller
        self.friendly_color_yellow = friendly_color_yellow

        # Setup Stop button for sending the STOP gamecontroller command
        self.stop_button = QPushButton()
        self.stop_button.setText("STOP")
        self.stop_button.setStyleSheet(self.get_button_style())
        self.stop_button.clicked.connect(self.__send_stop_command)

        # Setup Force Start button for sending the FORCE_START gamecontroller command
        self.force_start_button = QPushButton()
        self.force_start_button.setText("FORCE START")
        self.force_start_button.setStyleSheet(self.get_button_style())
        self.force_start_button.clicked.connect(self.__send_force_start_command)

        self.layout().addWidget(self.stop_button)
        self.layout().addWidget(self.force_start_button)
        self.layout().addStretch()

    def __send_stop_command(self):
        self.gamecontroller.send_gc_command(
            gc_command=Command.Type.STOP, team=Team.UNKNOWN
        )

    def __send_force_start_command(self):
        self.gamecontroller.send_gc_command(
            gc_command=Command.Type.FORCE_START,
            team=(Team.YELLOW if self.friendly_color_yellow else Team.BLUE),
        )
