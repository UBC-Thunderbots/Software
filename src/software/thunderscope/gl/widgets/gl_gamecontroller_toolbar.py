from software.thunderscope.gl.widgets.gl_toolbar import GLToolbar
from software.thunderscope.proto_unix_io import ProtoUnixIO
from pyqtgraph.Qt.QtWidgets import *
from proto.import_all_protos import *
from proto.ssl_gc_common_pb2 import Team as SslTeam


class GLGamecontrollerToolbar(GLToolbar):
    def __init__(self, proto_unix_io: ProtoUnixIO, friendly_color_yellow: bool):
        super(GLGamecontrollerToolbar, self).__init__()
        
        self.proto_unix_io = proto_unix_io
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
        self.__send_gc_command(
            Command.Type.STOP,
            SslTeam.UNKNOWN
        )

    def __send_force_start_command(self):
        print(self.friendly_color_yellow)
        self.__send_gc_command(
            Command.Type.FORCE_START,
            (SslTeam.YELLOW if self.friendly_color_yellow else SslTeam.BLUE)
        )

    def __send_gc_command(self, command: Command.Type, team: Team, ball_pos: Vector2 = None): 
        self.proto_unix_io.send_proto(
            ManualGCCommand, 
            ManualGCCommand(
                manual_command=Command(
                    type=command,
                    for_team=team
                ),
                final_ball_placement_point=ball_pos
            )
        )