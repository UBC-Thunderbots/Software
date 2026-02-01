from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt import QtGui
from proto.import_all_protos import *
from proto.ssl_gc_common_pb2 import Team as SslTeam
from software.thunderscope.gl.widgets.gl_toolbar import GLToolbar
import webbrowser
import qtawesome as qta
from software.thunderscope.proto_unix_io import ProtoUnixIO


class GamecontrollerPlays:
    """The different plays that can be set for each team"""

    NONE = "None"
    DIRECT = "Direct"
    INDIRECT = "Indirect"
    KICKOFF = "Kickoff"
    PENALTY = "Penalty"


class GamecontrollerEvents:
    """The different event that we can send for each team"""

    YELLOW = "Yellow Card"
    RED = "Red Card"
    GOAL = "Goal"
    TIMEOUT = "Start Timeout"


class GLGamecontrollerToolbar:
    """A toolbar with controls to send GameController commands from Thunderscope"""

    GAME_CONTROLLER_URL = "http://localhost:8081"

    def __init__(
        self,
        toolbar: GLToolbar,
        proto_unix_io: ProtoUnixIO,
        friendly_color_yellow: bool,
    ):
        self.proto_unix_io = proto_unix_io
        self.friendly_color_yellow = friendly_color_yellow

        self.toolbar = toolbar

        # Setup Stop button for sending the STOP gamecontroller command
        self.stop_button = self.toolbar.setup_icon_button(
            qta.icon("fa6s.pause"),
            "Stops gameplay, robots form circle around ball",
            self.__send_stop_command,
        )

        # Setup Force Start button for sending the FORCE_START gamecontroller command
        self.force_start_button = self.toolbar.setup_icon_button(
            qta.icon("ph.arrow-u-up-right-fill"),
            "Force Start, restarts the game",
            self.__send_force_start_command,
        )

        # Setup Halt button for sending the HALT gamecontroller command
        self.halt_button = self.toolbar.setup_icon_button(
            qta.icon("fa5s.stop"),
            "Halt, stops all robots immediately",
            self.__send_halt_command,
        )

        # Setup Normal Start button for sending the NORMAL_START gamecontroller command
        self.normal_start_button = self.toolbar.setup_icon_button(
            qta.icon("fa5s.play"),
            "Normal Start, resumes game from a set play (disabled when no play selected)",
            self.__send_normal_start_command,
        )

        # set up the menu for selecting plays
        self.plays_menu = QMenu()

        self.plays_menu_button = QPushButton()
        self.plays_menu_button.setText("Plays")
        self.plays_menu_button.setStyleSheet(self.toolbar.get_button_style())
        self.plays_menu_button.setMenu(self.plays_menu)

        # add play items for each team color
        self.__add_plays_menu_items(is_blue=True)
        self.plays_menu.addSeparator()
        self.__add_plays_menu_items(is_blue=False)

        self.gc_browser_button = self.toolbar.setup_icon_button(
            qta.icon("mdi6.open-in-new"),
            "Opens the SSL Gamecontroller in a browser window",
            lambda: webbrowser.open(self.GAME_CONTROLLER_URL, new=0, autoraise=True),
            display_text="Open GC",
        )

        # disable the normal start button when no play is selected
        self.normal_start_enabled = True
        self.__toggle_normal_start_button()

        self.toolbar.add_label("<b>Gamecontroller</b>")
        self.toolbar.add_separator()
        self.toolbar.add_button(self.stop_button)
        self.toolbar.add_button(self.halt_button)
        self.toolbar.add_button(self.force_start_button)
        self.toolbar.add_separator()
        self.toolbar.add_button(self.plays_menu_button)
        self.toolbar.add_button(self.normal_start_button)
        self.toolbar.add_separator()
        self.toolbar.add_button(self.gc_browser_button)

    def __add_plays_menu_items(self, is_blue: bool) -> None:
        """Initializes the plays menu with the available plays for the given team

        :param is_blue: if the team to add items for is blue (True) or yellow (False)
        """
        for arg in [
            GamecontrollerPlays.DIRECT,
            GamecontrollerPlays.INDIRECT,
            GamecontrollerPlays.KICKOFF,
            GamecontrollerPlays.PENALTY,
        ]:
            icon = qta.icon(
                "mdi6.square-rounded", color="blue" if is_blue else "yellow"
            )
            self.plays_menu.addAction(
                icon,
                arg,
                lambda play=arg: self.__plays_menu_handler(play, icon, is_blue),
            )

    def __plays_menu_handler(
        self, play: GamecontrollerPlays, icon: QtGui.QIcon, is_blue: bool
    ) -> None:
        """The handler called when a play is selected. Sends the right gc command
        based on the play we want.
        Updates the UI to indicate the play that was selected
        Toggles the normal start button so we can start the play
        """
        self.plays_menu_button.setIcon(icon)
        self.plays_menu_button.setText(play)

        command_type = None
        if play == GamecontrollerPlays.DIRECT:
            command_type = Command.Type.DIRECT
        elif play == GamecontrollerPlays.INDIRECT:
            command_type = Command.Type.INDIRECT
        elif play == GamecontrollerPlays.KICKOFF:
            command_type = Command.Type.KICKOFF
        else:
            command_type = Command.Type.PENALTY

        self.__send_gc_command(
            command_type,
            (SslTeam.BLUE if is_blue else SslTeam.YELLOW),
        )

        if not self.normal_start_enabled:
            self.__toggle_normal_start_button()

    def __toggle_normal_start_button(self) -> None:
        """Toggles the enabled / disabled state of the Normal Start button"""
        self.normal_start_enabled = not self.normal_start_enabled
        self.normal_start_button.setStyleSheet(
            self.toolbar.get_button_style(self.normal_start_enabled)
        )
        self.normal_start_button.setIcon(
            qta.icon(
                "fa5s.play",
                color=self.toolbar.BUTTON_ICON_COLOR
                if self.normal_start_enabled
                else self.toolbar.DISABLED_BUTTON_ICON_COLOR,
            )
        )

    def __send_stop_command(self) -> None:
        """Sends a STOP command to the gamecontroller"""
        self.__send_gc_command(Command.Type.STOP, SslTeam.UNKNOWN)

    def __send_force_start_command(self) -> None:
        """Sends a FORCE_START command for the current friendly team to the gamecontroller"""
        self.__send_gc_command(Command.Type.FORCE_START, SslTeam.UNKNOWN)

    def __send_halt_command(self) -> None:
        """Sends a HALT command for the current friendly team to the gamecontroller"""
        self.__send_gc_command(Command.Type.HALT, SslTeam.UNKNOWN)

    def __send_normal_start_command(self) -> None:
        """Sends a NORMAL START command for the current friendly team to the gamecontroller
        And resets the plays menu selection
        """
        if self.normal_start_enabled:
            self.__send_gc_command(Command.Type.NORMAL_START, SslTeam.UNKNOWN)

            self.__toggle_normal_start_button()

            self.plays_menu_button.setText("Plays")
            self.plays_menu_button.setIcon(QtGui.QIcon())

    def __send_gc_command(self, command: Command.Type, team: Team) -> None:
        """Sends the given command to the gamecontroller for the given Team
        If ball_pos is defined, sets the ball position

        :param command the command to send
        :param team the team the command should be sent for (BLUE, YELLOW, or UNKNOWN)
        :param ball_pos if defined, sets the position of the ball on field
        """
        command = ManualGCCommand(manual_command=Command(type=command, for_team=team))
        self.proto_unix_io.send_proto(ManualGCCommand, command)
