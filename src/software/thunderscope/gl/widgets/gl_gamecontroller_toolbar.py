from software.thunderscope.gl.widgets.gl_toolbar import GLToolbar
from software.thunderscope.proto_unix_io import ProtoUnixIO
from pyqtgraph.Qt.QtWidgets import *
from proto.import_all_protos import *
from proto.ssl_gc_common_pb2 import Team as SslTeam


class GLGamecontrollerToolbar(GLToolbar):
    """
    A toolbar with controls to send GameController commands from Thunderscope
    """

    def __init__(self, proto_unix_io: ProtoUnixIO, friendly_color_yellow: bool):
        """
        Initializes the toolbar and constructs its layout

        :param proto_unix_io the ProtoUnixIO object to send the manual gamecontroller commands to
        :param friendly_color_yellow True if yellow is friendly team, False if not
        """
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
        """
        Sends a STOP command to the gamecontroller
        """
        self.__send_gc_command(Command.Type.STOP, SslTeam.UNKNOWN)

    def __send_force_start_command(self):
        """
        Sends a FORCE_START command for the current friendly team to the gamecontroller
        """
        self.__send_gc_command(
            Command.Type.FORCE_START,
            (SslTeam.YELLOW if self.friendly_color_yellow else SslTeam.BLUE),
        )

    def __send_gc_command(
        self, command: Command.Type, team: Team, ball_pos: Vector2 = None
    ):
        """
        Sends the given command to the gamecontroller for the given Team
        If ball_pos is defined, sets the ball position

        :param command the command to send
        :param team the team the command should be sent for (BLUE, YELLOW, or UNKNOWN)
        :param ball_pos if defined, sets the position of the ball on field
        """
        command = ManualGCCommand(manual_command=Command(type=command, for_team=team))

        # only set it if defined
        if ball_pos:
            command.final_ball_placement_point = ball_pos

        self.proto_unix_io.send_proto(ManualGCCommand, command)
